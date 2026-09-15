#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>

#define SDA PB7
#define SCL PB6

#define XF PA0
#define XL PA1
#define XR PA2
#define XB PA3

#define L1 PB0
#define L2 PB1
#define R1 PB10
#define R2 PB11

#define FA 0x30
#define LA 0x31
#define RA 0x32
#define BA 0x33

#define MPU 0x68

#define MS 16

#define N 0
#define E 1
#define S 2
#define W 3

#define WN 1
#define WE 2
#define WS 4
#define WW 8

#define BS 150
#define CT 700

#define FW 110
#define SW 120

VL53L0X f;
VL53L0X l;
VL53L0X r;
VL53L0X b;

struct Cell
{
  byte w;
  bool v;
};

struct Pt
{
  int x;
  int y;
};

Cell m[MS][MS];

Pt path[256];

int pl = 0;

int rx = 0;
int ry = 0;

byte rd = N;

int gx = 7;
int gy = 7;

float yaw = 0;
float gb = 0;

unsigned long lt;

int dx[4] = {0, 1, 0, -1};
int dy[4] = {1, 0, -1, 0};

byte wb[4] = {WN, WE, WS, WW};
byte op[4] = {S, W, N, E};


class KF
{
public:

  float a;
  float bias;

  float p00;
  float p01;
  float p10;
  float p11;

  float qa;
  float qb;
  float rm;

  KF()
  {
    a = 0;
    bias = 0;

    p00 = 0;
    p01 = 0;
    p10 = 0;
    p11 = 0;

    qa = 0.001;
    qb = 0.003;
    rm = 0.03;
  }

  float upd(float rate, float dt)
  {
    float rr = rate - bias;

    a += dt * rr;

    p00 += dt * (dt * p11 - p01 - p10 + qa);

    p01 -= dt * p11;
    p10 -= dt * p11;

    p11 += qb * dt;

    return a;
  }

  void set(float x)
  {
    a = x;
  }
};

KF k;


void mw(byte reg, byte val)
{
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}


bool mr(byte reg, byte *buf, byte len)
{
  Wire.beginTransmission(MPU);
  Wire.write(reg);

  if(Wire.endTransmission(false) != 0)
    return false;

  Wire.requestFrom(MPU, len);

  if(Wire.available() < len)
    return false;

  for(byte i = 0; i < len; i++)
    buf[i] = Wire.read();

  return true;
}


bool mi()
{
  byte id;

  if(!mr(0x75, &id, 1))
    return false;

  Serial.print("MPU: 0x");
  Serial.println(id, HEX);

  if(id != 0x70 && id != 0x71)
    return false;

  mw(0x6B, 0x00);
  delay(100);

  mw(0x1A, 0x03);
  mw(0x1B, 0x00);
  mw(0x1C, 0x00);

  return true;
}


float gz()
{
  byte d[2];

  if(!mr(0x47, d, 2))
    return 0;

  int16_t v = ((int16_t)d[0] << 8) | d[1];

  return v / 131.0;
}


void cal()
{
  Serial.println("Keep robot still");

  delay(1000);

  float s = 0;

  for(int i = 0; i < 1000; i++)
  {
    s += gz();
    delay(2);
  }

  gb = s / 1000.0;

  Serial.print("Bias: ");
  Serial.println(gb);
}


void imu()
{
  unsigned long t = micros();

  float dt = (t - lt) / 1000000.0;

  lt = t;

  if(dt <= 0 || dt > 0.1)
    return;

  float g = gz() - gb;

  yaw = k.upd(g, dt);

  while(yaw > 180)
    yaw -= 360;

  while(yaw < -180)
    yaw += 360;
}


float norm(float x)
{
  while(x > 180)
    x -= 360;

  while(x < -180)
    x += 360;

  return x;
}


void motorL(int x)
{
  x = constrain(x, -255, 255);

  if(x > 0)
  {
    analogWrite(L1, x);
    analogWrite(L2, 0);
  }
  else if(x < 0)
  {
    analogWrite(L1, 0);
    analogWrite(L2, -x);
  }
  else
  {
    analogWrite(L1, 0);
    analogWrite(L2, 0);
  }
}


void motorR(int x)
{
  x = constrain(x, -255, 255);

  if(x > 0)
  {
    analogWrite(R1, x);
    analogWrite(R2, 0);
  }
  else if(x < 0)
  {
    analogWrite(R1, 0);
    analogWrite(R2, -x);
  }
  else
  {
    analogWrite(R1, 0);
    analogWrite(R2, 0);
  }
}


void mot(int lsp, int rsp)
{
  motorL(lsp);
  motorR(rsp);
}


void stopm()
{
  mot(0, 0);
}


bool tof()
{
  pinMode(XF, OUTPUT);
  pinMode(XL, OUTPUT);
  pinMode(XR, OUTPUT);
  pinMode(XB, OUTPUT);

  digitalWrite(XF, LOW);
  digitalWrite(XL, LOW);
  digitalWrite(XR, LOW);
  digitalWrite(XB, LOW);

  delay(100);

  digitalWrite(XF, HIGH);
  delay(50);

  if(!f.init())
    return false;

  f.setAddress(FA);
  f.setTimeout(100);
  f.startContinuous();

  digitalWrite(XL, HIGH);
  delay(50);

  if(!l.init())
    return false;

  l.setAddress(LA);
  l.setTimeout(100);
  l.startContinuous();

  digitalWrite(XR, HIGH);
  delay(50);

  if(!r.init())
    return false;

  r.setAddress(RA);
  r.setTimeout(100);
  r.startContinuous();

  digitalWrite(XB, HIGH);
  delay(50);

  if(!b.init())
    return false;

  b.setAddress(BA);
  b.setTimeout(100);
  b.startContinuous();

  return true;
}


int rf()
{
  return f.readRangeContinuousMillimeters();
}


int rl()
{
  return l.readRangeContinuousMillimeters();
}


int rr()
{
  return r.readRangeContinuousMillimeters();
}


int rb()
{
  return b.readRangeContinuousMillimeters();
}


void sw(int x, int y, byte d)
{
  if(x < 0 || x >= MS || y < 0 || y >= MS)
    return;

  m[y][x].w |= wb[d];

  int nx = x + dx[d];
  int ny = y + dy[d];

  if(nx >= 0 && nx < MS && ny >= 0 && ny < MS)
    m[ny][nx].w |= wb[op[d]];
}


bool hw(int x, int y, byte d)
{
  if(x < 0 || x >= MS || y < 0 || y >= MS)
    return true;

  return m[y][x].w & wb[d];
}


void mapw()
{
  int f1 = rf();
  int l1 = rl();
  int r1 = rr();
  int b1 = rb();

  byte fd = rd;
  byte rd1 = (rd + 1) % 4;
  byte bd = (rd + 2) % 4;
  byte ld = (rd + 3) % 4;

  if(f1 < FW)
    sw(rx, ry, fd);

  if(l1 < SW)
    sw(rx, ry, ld);

  if(r1 < SW)
    sw(rx, ry, rd1);

  if(b1 < SW)
    sw(rx, ry, bd);

  m[ry][rx].v = true;
}


void turn(float deg)
{
  float tar = norm(yaw + deg);

  unsigned long t0 = millis();

  while(1)
  {
    imu();

    float e = norm(tar - yaw);

    if(fabs(e) < 2)
      break;

    int sp = constrain((int)(e * 4), -180, 180);

    mot(sp, -sp);

    if(millis() - t0 > 3000)
      break;

    delay(5);
  }

  stopm();

  delay(100);

  yaw = tar;
  k.set(tar);
}


void face(byte d)
{
  int e = d - rd;

  if(e > 2)
    e -= 4;

  if(e < -2)
    e += 4;

  if(e == 1)
    turn(90);

  else if(e == -1)
    turn(-90);

  else if(abs(e) == 2)
    turn(180);

  rd = d;
}


void moveCell()
{
  unsigned long t0 = millis();

  float hd = yaw;

  while(millis() - t0 < CT)
  {
    imu();

    float e = norm(hd - yaw);

    int c = constrain((int)(e * 4), -60, 60);

    int ls = BS + c;
    int rs = BS - c;

    if(rf() < 60)
    {
      stopm();
      return;
    }

    mot(ls, rs);

    delay(5);
  }

  stopm();
  delay(100);
}


void pos()
{
  if(rd == N)
    ry++;

  else if(rd == E)
    rx++;

  else if(rd == S)
    ry--;

  else if(rd == W)
    rx--;

  rx = constrain(rx, 0, MS - 1);
  ry = constrain(ry, 0, MS - 1);
}


int h(int x, int y)
{
  return abs(x - gx) + abs(y - gy);
}


bool astar(
  int sx,
  int sy,
  int ex,
  int ey
)
{
  static int g[MS][MS];
  static int ff[MS][MS];

  static bool open[MS][MS];
  static bool close[MS][MS];

  static Pt par[MS][MS];

  for(int y = 0; y < MS; y++)
  {
    for(int x = 0; x < MS; x++)
    {
      g[y][x] = 100000;
      ff[y][x] = 100000;

      open[y][x] = false;
      close[y][x] = false;

      par[y][x].x = -1;
      par[y][x].y = -1;
    }
  }

  g[sy][sx] = 0;
  ff[sy][sx] = h(sx, sy);
  open[sy][sx] = true;

  while(1)
  {
    int cx = -1;
    int cy = -1;
    int bf = 100000;

    for(int y = 0; y < MS; y++)
    {
      for(int x = 0; x < MS; x++)
      {
        if(
          open[y][x] &&
          !close[y][x] &&
          ff[y][x] < bf
        )
        {
          bf = ff[y][x];
          cx = x;
          cy = y;
        }
      }
    }

    if(cx == -1)
      return false;

    if(cx == ex && cy == ey)
      break;

    open[cy][cx] = false;
    close[cy][cx] = true;

    for(int d = 0; d < 4; d++)
    {
      if(hw(cx, cy, d))
        continue;

      int nx = cx + dx[d];
      int ny = cy + dy[d];

      if(
        nx < 0 ||
        nx >= MS ||
        ny < 0 ||
        ny >= MS
      )
        continue;

      if(close[ny][nx])
        continue;

      int ng = g[cy][cx] + 1;

      if(!open[ny][nx] || ng < g[ny][nx])
      {
        par[ny][nx].x = cx;
        par[ny][nx].y = cy;

        g[ny][nx] = ng;

        ff[ny][nx] =
          ng + h(nx, ny);

        open[ny][nx] = true;
      }
    }
  }

  pl = 0;

  Pt cur;
  cur.x = ex;
  cur.y = ey;

  while(pl < 256)
  {
    path[pl++] = cur;

    if(cur.x == sx && cur.y == sy)
      break;

    cur = par[cur.y][cur.x];
  }

  for(int i = 0; i < pl / 2; i++)
  {
    Pt t = path[i];

    path[i] =
      path[pl - 1 - i];

    path[pl - 1 - i] = t;
  }

  return true;
}


byte dirTo(Pt a, Pt b)
{
  if(b.x > a.x)
    return E;

  if(b.x < a.x)
    return W;

  if(b.y > a.y)
    return N;

  return S;
}


void runPath()
{
  for(int i = 1; i < pl; i++)
  {
    byte d =
      dirTo(
        path[i - 1],
        path[i]
      );

    face(d);

    moveCell();

    rx = path[i].x;
    ry = path[i].y;
  }

  stopm();
}


byte chooseDir(
  int fv,
  int lv,
  int rv,
  int bv
)
{
  byte fd = rd;
  byte rd1 = (rd + 1) % 4;
  byte bd = (rd + 2) % 4;
  byte ld = (rd + 3) % 4;

  int nx;
  int ny;

  nx = rx + dx[fd];
  ny = ry + dy[fd];

  if(
    fv > 140 &&
    nx >= 0 &&
    nx < MS &&
    ny >= 0 &&
    ny < MS &&
    !m[ny][nx].v
  )
    return fd;


  nx = rx + dx[ld];
  ny = ry + dy[ld];

  if(
    lv > 150 &&
    nx >= 0 &&
    nx < MS &&
    ny >= 0 &&
    ny < MS &&
    !m[ny][nx].v
  )
    return ld;


  nx = rx + dx[rd1];
  ny = ry + dy[rd1];

  if(
    rv > 150 &&
    nx >= 0 &&
    nx < MS &&
    ny >= 0 &&
    ny < MS &&
    !m[ny][nx].v
  )
    return rd1;


  nx = rx + dx[bd];
  ny = ry + dy[bd];

  if(
    bv > 150 &&
    nx >= 0 &&
    nx < MS &&
    ny >= 0 &&
    ny < MS &&
    !m[ny][nx].v
  )
    return bd;


  if(fv > 140)
    return fd;

  if(lv > 150)
    return ld;

  if(rv > 150)
    return rd1;

  return bd;
}


void explore()
{
  int fv = rf();
  int lv = rl();
  int rv = rr();
  int bv = rb();

  mapw();

  byte d =
    chooseDir(
      fv,
      lv,
      rv,
      bv
    );

  face(d);

  moveCell();

  pos();
}


void setup()
{
  Serial.begin(115200);

  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);

  stopm();

  Wire.setSDA(SDA);
  Wire.setSCL(SCL);
  Wire.begin();
  Wire.setClock(400000);

  if(!mi())
  {
    Serial.println("MPU error");

    while(1)
    {
      stopm();
      delay(1000);
    }
  }

  cal();

  k.set(0);

  yaw = 0;

  lt = micros();

  if(!tof())
  {
    Serial.println("ToF error");

    while(1)
    {
      stopm();
      delay(1000);
    }
  }

  for(int y = 0; y < MS; y++)
  {
    for(int x = 0; x < MS; x++)
    {
      m[y][x].w = 0;
      m[y][x].v = false;
    }
  }

  for(int x = 0; x < MS; x++)
  {
    m[0][x].w |= WS;
    m[MS - 1][x].w |= WN;
  }

  for(int y = 0; y < MS; y++)
  {
    m[y][0].w |= WW;
    m[y][MS - 1].w |= WE;
  }

  rx = 0;
  ry = 0;
  rd = N;

  delay(3000);

  Serial.println("Ready");
}


void loop()
{
  if(
    rx == gx &&
    ry == gy
  )
  {
    stopm();

    Serial.println("Goal found");

    delay(1000);

    if(
      astar(
        0,
        0,
        gx,
        gy
      )
    )
    {
      Serial.print("Path: ");

      for(int i = 0; i < pl; i++)
      {
        Serial.print("(");
        Serial.print(path[i].x);
        Serial.print(",");
        Serial.print(path[i].y);
        Serial.print(") ");
      }

      Serial.println();

      while(1)
      {
        stopm();
        delay(1000);
      }
    }

    while(1)
    {
      stopm();
      delay(1000);
    }
  }

  explore();

  delay(30);
}