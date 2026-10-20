// Стол Мастера — демонстратор технологий ролевого стола.
// C++17, freestanding, собирается clang'ом напрямую в WebAssembly (без Emscripten).
// Весь кадр рисуется здесь: процедурное подземелье, динамический свет с тенями,
// туман войны, A*, частицы и 3D-кость d20 с физикой. JS только копирует кадр в canvas.

typedef unsigned int u32;
typedef unsigned char u8;
typedef __SIZE_TYPE__ usize;

#define EXPORT(name) extern "C" __attribute__((export_name(#name)))

extern "C" void* memset(void* d, int c, usize n) {
    u8* p = (u8*)d;
    while (n--) *p++ = (u8)c;
    return d;
}
extern "C" void* memcpy(void* d, const void* s, usize n) {
    u8* p = (u8*)d;
    const u8* q = (const u8*)s;
    while (n--) *p++ = *q++;
    return d;
}

// ---------------------------------------------------------------- math

static const float PI = 3.14159265f;
static inline float fsqrt(float x) { return __builtin_sqrtf(x); }
static inline float ffloor(float x) { return __builtin_floorf(x); }
static inline float fabs_(float x) { return __builtin_fabsf(x); }
static inline float fmin_(float a, float b) { return a < b ? a : b; }
static inline float fmax_(float a, float b) { return a > b ? a : b; }
static inline float sat(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }
static inline float smooth(float x) { x = sat(x); return x * x * (3 - 2 * x); }
static inline float mix(float a, float b, float t) { return a + (b - a) * t; }

static float fsin(float x) {
    x = x - 6.2831853f * ffloor((x + PI) / 6.2831853f);
    if (x > PI * 0.5f) x = PI - x;
    else if (x < -PI * 0.5f) x = -PI - x;
    float x2 = x * x;
    return x * (1.f + x2 * (-1.f / 6 + x2 * (1.f / 120 + x2 * (-1.f / 5040 + x2 * (1.f / 362880)))));
}
static inline float fcos(float x) { return fsin(x + PI * 0.5f); }

static float fatan2(float y, float x) {
    float ax = fabs_(x), ay = fabs_(y);
    float mx = ax > ay ? ax : ay, mn = ax > ay ? ay : ax;
    float a = mn / (mx + 1e-9f);
    float s = a * a;
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    if (ay > ax) r = 1.57079637f - r;
    if (x < 0) r = PI - r;
    if (y < 0) r = -r;
    return r;
}
static inline float fexp_neg(float x) {  // e^-x for x>=0, достаточно точно для затухания
    float t = 1.f + x * (1.f / 16);
    t *= t; t *= t; t *= t; t *= t;
    return 1.f / t;
}

struct V3 { float x, y, z; };
static inline V3 v3(float x, float y, float z) { V3 r = {x, y, z}; return r; }
static inline V3 add(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline V3 sub(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline V3 mul(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 cross(V3 a, V3 b) { return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
static inline V3 norm(V3 a) { float l = fsqrt(dot(a, a)); return l > 1e-9f ? mul(a, 1.f / l) : a; }

struct Quat { float w, x, y, z; };
static inline Quat qmul(Quat a, Quat b) {
    Quat r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return r;
}
static inline Quat qnorm(Quat q) {
    float l = fsqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w /= l; q.x /= l; q.y /= l; q.z /= l;
    return q;
}
static inline Quat qaxis(V3 axis, float ang) {
    float s = fsin(ang * 0.5f);
    Quat q = {fcos(ang * 0.5f), axis.x * s, axis.y * s, axis.z * s};
    return q;
}
static inline V3 qrot(Quat q, V3 v) {
    V3 u = v3(q.x, q.y, q.z);
    V3 t = mul(cross(u, v), 2.f);
    return add(add(v, mul(t, q.w)), cross(u, t));
}

// ---------------------------------------------------------------- random

static u32 rng_state = 1;
static u32 rnd() {
    u32 x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return rng_state = x;
}
static float rndf() { return (rnd() >> 8) * (1.f / 16777216.f); }
static int rndi(int a, int b) { return a + (int)(rnd() % (u32)(b - a + 1)); }
static inline u32 hash(u32 x) {
    x ^= x >> 16; x *= 0x7feb352du; x ^= x >> 15; x *= 0x846ca68bu; x ^= x >> 16;
    return x;
}
static inline u32 hash2(int x, int y) { return hash((u32)x * 73856093u ^ (u32)y * 19349663u); }
static inline float hashf(int x, int y) { return (hash2(x, y) & 0xffffff) * (1.f / 16777216.f); }

// ---------------------------------------------------------------- map

enum { MW = 56, MH = 56, LS = 4, LW = MW * LS, LH = MH * LS };
enum { ROCK = 0, FLOOR = 1 };

static u8 tiles[MH][MW];
static u8 nmask[MH][MW];
static float tileHash[MH][MW];

static inline bool solid(int x, int y) {
    if (x < 0 || y < 0 || x >= MW || y >= MH) return true;
    return tiles[y][x] == ROCK;
}

struct Room { int x, y, w, h; };
static Room rooms[24];
static int nroom;

enum { L_TORCH = 0, L_CRYSTAL = 1 };
struct Light {
    float x, y, r, g, b, rad, inten, phase;
    int kind;
    u32 start, count;
    float spawn;
};
static Light lights[64];
static int nlight;

struct Splat { u32 idx; float w; };
enum { POOL = 260000 };
static Splat pool[POOL];
static u32 poolN;

enum { M_GOBLIN = 1, M_SKELETON = 2 };
struct Monster { float x, y; int type; bool alive; float hit, phase, death; };
static Monster mons[40];
static int nmon;

// свет: r,g,b (уже умножены на видимость), m — вес «памяти» тумана
static float lm[LW * LH * 4];
static float visA[LW * LH];
static float expl[LW * LH];

// ---------------------------------------------------------------- state

enum { MAXW = 2048, MAXH = 2048 };
static u32 fb[MAXW * MAXH];
static int FW = 0, FH = 0;

static float camX = 10, camY = 10, zoom = 40;
static bool follow = true;
static float heroX, heroY;
static int path[1024], pathLen, pathIdx;
static int attackTarget = -1, pendingAttack = -1;
static bool fogOn = true, lanternOn = true;
static float timeNow = 0;
static float heroPhase = 0;

struct Particle { float x, y, vx, vy, life, max, r, g, b, size; };
enum { MAXP = 900 };
static Particle parts[MAXP];
static int nparts;

struct Ripple { float x, y, t; bool bad; };
static Ripple ripples[8];
static int nrip;

// ---------------------------------------------------------------- line of sight (DDA)

static bool los(float x0, float y0, float x1, float y1) {
    int tx = (int)ffloor(x1), ty = (int)ffloor(y1);
    int x = (int)ffloor(x0), y = (int)ffloor(y0);
    float dx = x1 - x0, dy = y1 - y0;
    int sx = dx > 0 ? 1 : -1, sy = dy > 0 ? 1 : -1;
    float tdx = dx != 0 ? fabs_(1.f / dx) : 1e30f;
    float tdy = dy != 0 ? fabs_(1.f / dy) : 1e30f;
    float tmx = dx != 0 ? (dx > 0 ? (x + 1 - x0) : (x0 - x)) * tdx : 1e30f;
    float tmy = dy != 0 ? (dy > 0 ? (y + 1 - y0) : (y0 - y)) * tdy : 1e30f;
    for (int i = 0; i < 128; i++) {
        if (x == tx && y == ty) return true;
        if (i > 0 && solid(x, y)) return false;
        if (tmx > 1.f && tmy > 1.f) return true;
        if (tmx < tmy) { tmx += tdx; x += sx; }
        else { tmy += tdy; y += sy; }
    }
    return true;
}

// ---------------------------------------------------------------- dungeon generation

static void carve_rect(int x, int y, int w, int h) {
    for (int j = y; j < y + h; j++)
        for (int i = x; i < x + w; i++)
            if (i > 0 && j > 0 && i < MW - 1 && j < MH - 1) tiles[j][i] = FLOOR;
}
static void carve_h(int x1, int x2, int y) {
    if (x1 > x2) { int t = x1; x1 = x2; x2 = t; }
    carve_rect(x1, y, x2 - x1 + 1, 1);
}
static void carve_v(int y1, int y2, int x) {
    if (y1 > y2) { int t = y1; y1 = y2; y2 = t; }
    carve_rect(x, y1, 1, y2 - y1 + 1);
}
static void connect(const Room& a, const Room& b) {
    int ax = a.x + a.w / 2, ay = a.y + a.h / 2;
    int bx = b.x + b.w / 2, by = b.y + b.h / 2;
    if (rnd() & 1) { carve_h(ax, bx, ay); carve_v(ay, by, bx); }
    else { carve_v(ay, by, ax); carve_h(ax, bx, by); }
}

static void bake_light(Light& L) {
    L.start = poolN;
    int R = (int)(L.rad * LS) + 1;
    int cx = (int)(L.x * LS), cy = (int)(L.y * LS);
    for (int y = cy - R; y <= cy + R; y++) {
        if (y < 0 || y >= LH) continue;
        for (int x = cx - R; x <= cx + R; x++) {
            if (x < 0 || x >= LW) continue;
            float wx = (x + 0.5f) / LS, wy = (y + 0.5f) / LS;
            float dx = wx - L.x, dy = wy - L.y;
            float d = fsqrt(dx * dx + dy * dy);
            if (d >= L.rad) continue;
            if (!los(L.x, L.y, wx, wy)) continue;
            if (poolN >= POOL) break;
            float a = 1.f - d / L.rad;
            pool[poolN].idx = (u32)(y * LW + x);
            pool[poolN].w = (a * a + 0.35f * a * a * a * a) * L.inten;
            poolN++;
        }
    }
    L.count = poolN - L.start;
}

static void add_light(float x, float y, float r, float g, float b, float rad, float inten, int kind) {
    if (nlight >= 64) return;
    Light& L = lights[nlight++];
    L.x = x; L.y = y; L.r = r; L.g = g; L.b = b;
    L.rad = rad; L.inten = inten; L.kind = kind;
    L.phase = rndf() * 100.f;
    L.spawn = 0;
    bake_light(L);
}

static bool tile_free(int x, int y) {
    if (solid(x, y)) return false;
    for (int i = 0; i < nmon; i++)
        if ((int)mons[i].x == x && (int)mons[i].y == y) return false;
    return true;
}

static void generate(u32 seed) {
    rng_state = seed ? seed : 0x9e3779b9u;
    for (int i = 0; i < 8; i++) rnd();
    memset(tiles, 0, sizeof(tiles));
    nroom = 0; nlight = 0; poolN = 0; nmon = 0;
    nparts = 0; nrip = 0; pathLen = pathIdx = 0;
    attackTarget = pendingAttack = -1;

    for (int a = 0; a < 600 && nroom < 15; a++) {
        int w = rndi(5, 10), h = rndi(5, 9);
        int x = rndi(2, MW - w - 3), y = rndi(2, MH - h - 3);
        bool ok = true;
        for (int i = 0; i < nroom; i++) {
            Room& r = rooms[i];
            if (x - 2 < r.x + r.w && x + w + 2 > r.x && y - 2 < r.y + r.h && y + h + 2 > r.y) { ok = false; break; }
        }
        if (!ok) continue;
        Room r = {x, y, w, h};
        rooms[nroom++] = r;
        carve_rect(x, y, w, h);
        if (w >= 8 && h >= 7) {  // колонны — чтобы тени были интереснее
            tiles[y + 2][x + 2] = ROCK; tiles[y + 2][x + w - 3] = ROCK;
            tiles[y + h - 3][x + 2] = ROCK; tiles[y + h - 3][x + w - 3] = ROCK;
        }
    }
    for (int i = 1; i < nroom; i++) {
        int best = 0; float bd = 1e9f;
        for (int j = 0; j < i; j++) {
            float dx = (float)(rooms[i].x - rooms[j].x), dy = (float)(rooms[i].y - rooms[j].y);
            float d = dx * dx + dy * dy;
            if (d < bd) { bd = d; best = j; }
        }
        connect(rooms[i], rooms[best]);
    }
    for (int k = 0; k < 3 && nroom > 3; k++) connect(rooms[rndi(0, nroom - 1)], rooms[rndi(0, nroom - 1)]);

    for (int y = 0; y < MH; y++)
        for (int x = 0; x < MW; x++) {
            tileHash[y][x] = hashf(x * 7 + 3, y * 13 + 5);
            u8 m = 0;
            if (tiles[y][x] == FLOOR) {
                if (solid(x - 1, y)) m |= 1;
                if (solid(x + 1, y)) m |= 2;
                if (solid(x, y - 1)) m |= 4;
                if (solid(x, y + 1)) m |= 8;
            } else {
                if (!solid(x - 1, y)) m |= 1;
                if (!solid(x + 1, y)) m |= 2;
                if (!solid(x, y - 1)) m |= 4;
                if (!solid(x, y + 1)) m |= 8;
                for (int j = -1; j <= 1; j++)
                    for (int i = -1; i <= 1; i++)
                        if (!solid(x + i, y + j)) m |= 16;
            }
            nmask[y][x] = m;
        }

    // факелы на верхних и нижних стенах комнат, кристаллы — цветной свет
    for (int r = 0; r < nroom; r++) {
        Room& R = rooms[r];
        int placed = 0;
        for (int t = 0; t < 12 && placed < 2; t++) {
            int x = rndi(R.x, R.x + R.w - 1);
            bool top = (t + r) & 1;
            int wy = top ? R.y - 1 : R.y + R.h;
            int fy = top ? R.y : R.y + R.h - 1;
            if (!solid(x, wy) || solid(x, fy)) continue;
            bool near = false;
            for (int i = 0; i < nlight; i++) {
                float dx = lights[i].x - (x + 0.5f), dy = lights[i].y - (fy + 0.5f);
                if (dx * dx + dy * dy < 9) near = true;
            }
            if (near) continue;
            add_light(x + 0.5f, top ? fy + 0.12f : fy + 0.88f, 1.0f, 0.55f, 0.22f, 6.5f, 1.55f, L_TORCH);
            placed++;
        }
        if (r > 0 && rndi(0, 2) == 0) {
            int x = rndi(R.x + 1, R.x + R.w - 2), y = rndi(R.y + 1, R.y + R.h - 2);
            if (!solid(x, y)) {
                bool cyan = rnd() & 1;
                add_light(x + 0.5f, y + 0.5f, cyan ? 0.25f : 0.8f, cyan ? 0.75f : 0.3f, 1.0f, 5.0f, 1.2f, L_CRYSTAL);
                tiles[y][x] = FLOOR;
            }
        }
    }

    Room& R0 = rooms[0];
    heroX = R0.x + R0.w / 2 + 0.5f;
    heroY = R0.y + R0.h / 2 + 0.5f;
    camX = heroX; camY = heroY; follow = true;

    for (int r = 1; r < nroom; r++) {
        int cnt = rndi(1, 2);
        for (int k = 0; k < cnt && nmon < 40; k++) {
            for (int t = 0; t < 20; t++) {
                int x = rndi(rooms[r].x, rooms[r].x + rooms[r].w - 1);
                int y = rndi(rooms[r].y, rooms[r].y + rooms[r].h - 1);
                if (!tile_free(x, y)) continue;
                bool onLight = false;
                for (int i = 0; i < nlight; i++)
                    if ((int)lights[i].x == x && (int)lights[i].y == y && lights[i].kind == L_CRYSTAL) onLight = true;
                if (onLight) continue;
                Monster& m = mons[nmon++];
                m.x = x + 0.5f; m.y = y + 0.5f;
                m.type = rndi(0, 3) == 0 ? M_SKELETON : M_GOBLIN;
                m.alive = true; m.hit = 0; m.death = 0; m.phase = rndf() * 6.28f;
                break;
            }
        }
    }
    memset(expl, 0, sizeof(expl));
}

// ---------------------------------------------------------------- A*

static float gScore[MW * MH];
static int cameFrom[MW * MH];
static u8 nodeState[MW * MH];  // 0 new, 1 open, 2 closed
static int openList[MW * MH];

static bool blocked_by_monster(int x, int y) {
    for (int i = 0; i < nmon; i++)
        if (mons[i].alive && (int)mons[i].x == x && (int)mons[i].y == y) return true;
    return false;
}

static bool astar(int sx, int sy, int gx, int gy) {
    if (solid(gx, gy)) return false;
    memset(nodeState, 0, sizeof(nodeState));
    int start = sy * MW + sx, goal = gy * MW + gx;
    int nopen = 0;
    gScore[start] = 0; cameFrom[start] = -1; nodeState[start] = 1;
    openList[nopen++] = start;
    while (nopen > 0) {
        int bi = 0; float bf = 1e30f;
        for (int i = 0; i < nopen; i++) {
            int n = openList[i];
            int nx = n % MW, ny = n / MW;
            float dx = fabs_((float)(nx - gx)), dy = fabs_((float)(ny - gy));
            float h = (dx + dy) + (1.41421f - 2.f) * fmin_(dx, dy);
            float f = gScore[n] + h;
            if (f < bf) { bf = f; bi = i; }
        }
        int cur = openList[bi];
        openList[bi] = openList[--nopen];
        nodeState[cur] = 2;
        if (cur == goal) {
            int tmp[MW * MH / 2]; int n = 0;
            for (int c = goal; c != -1 && n < MW * MH / 2; c = cameFrom[c]) tmp[n++] = c;
            pathLen = 0;
            for (int i = n - 2; i >= 0 && pathLen < 1024; i--) path[pathLen++] = tmp[i];
            pathIdx = 0;
            return true;
        }
        int cx = cur % MW, cy = cur / MW;
        for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++) {
                if (!dx && !dy) continue;
                int nx = cx + dx, ny = cy + dy;
                if (solid(nx, ny)) continue;
                if (dx && dy && (solid(cx + dx, cy) || solid(cx, cy + dy))) continue;
                int ni = ny * MW + nx;
                if (ni != goal && blocked_by_monster(nx, ny)) continue;
                if (nodeState[ni] == 2) continue;
                float ng = gScore[cur] + ((dx && dy) ? 1.41421f : 1.f);
                if (nodeState[ni] == 1 && ng >= gScore[ni]) continue;
                gScore[ni] = ng; cameFrom[ni] = cur;
                if (nodeState[ni] != 1) { nodeState[ni] = 1; openList[nopen++] = ni; }
            }
    }
    return false;
}

// ---------------------------------------------------------------- particles

static void spawn(float x, float y, float vx, float vy, float life, float r, float g, float b, float size) {
    if (nparts >= MAXP) return;
    Particle& p = parts[nparts++];
    p.x = x; p.y = y; p.vx = vx; p.vy = vy; p.life = life; p.max = life;
    p.r = r; p.g = g; p.b = b; p.size = size;
}

static void burst(float x, float y, float r, float g, float b, int n) {
    for (int i = 0; i < n; i++) {
        float a = rndf() * 6.2832f, s = 0.8f + rndf() * 2.6f;
        spawn(x, y, fcos(a) * s, fsin(a) * s, 0.5f + rndf() * 0.7f, r, g, b, 0.05f + rndf() * 0.06f);
    }
}

// ---------------------------------------------------------------- d20

static V3 icoV[12];
static int icoF[20][3];
static int faceNum[20];
static V3 faceN[20];

struct Dice {
    bool active, settled, attack;
    float x, y, vx, vy, t, scale;
    Quat q;
    V3 w;
    int result, top, reported, event;
} dice;

static void init_ico() {
    const float p = 1.6180339f;
    float raw[12][3] = {{-1, p, 0}, {1, p, 0}, {-1, -p, 0}, {1, -p, 0}, {0, -1, p}, {0, 1, p},
                        {0, -1, -p}, {0, 1, -p}, {p, 0, -1}, {p, 0, 1}, {-p, 0, -1}, {-p, 0, 1}};
    int f[20][3] = {{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4},
                    {11, 10, 2}, {10, 7, 6}, {7, 1, 8}, {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8},
                    {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
    for (int i = 0; i < 12; i++) icoV[i] = norm(v3(raw[i][0], raw[i][1], raw[i][2]));
    for (int i = 0; i < 20; i++) {
        for (int k = 0; k < 3; k++) icoF[i][k] = f[i][k];
        faceN[i] = norm(add(add(icoV[f[i][0]], icoV[f[i][1]]), icoV[f[i][2]]));
        faceNum[i] = 0;
    }
    // противоположные грани в сумме дают 21, как у настоящей кости
    int k = 1;
    for (int i = 0; i < 20; i++) {
        if (faceNum[i]) continue;
        int opp = 0; float best = 2;
        for (int j = 0; j < 20; j++) {
            float d = dot(faceN[i], faceN[j]);
            if (d < best) { best = d; opp = j; }
        }
        int order[10] = {20, 1, 14, 7, 17, 3, 11, 9, 16, 6};
        int n = order[(k - 1) % 10];
        faceNum[i] = n; faceNum[opp] = 21 - n;
        k++;
    }
}

static void roll_dice(bool attack) {
    float S = (float)(FW < FH ? FW : FH);
    dice.active = true; dice.settled = false; dice.attack = attack;
    dice.x = FW * (0.3f + 0.4f * rndf());
    dice.y = FH * 0.92f;
    dice.vx = (rndf() - 0.5f) * S * 1.6f;
    dice.vy = -S * (1.5f + rndf() * 0.8f);
    dice.t = 0; dice.scale = 1;
    Quat q = {1, rndf() - 0.5f, rndf() - 0.5f, rndf() - 0.5f};
    dice.q = qnorm(q);
    dice.w = v3((rndf() - 0.5f) * 30, (rndf() - 0.5f) * 30, (rndf() - 0.5f) * 14);
    dice.result = 0; dice.reported = 0; dice.event = 0;
}

static int dice_top() {
    int best = 0; float bz = -2;
    for (int i = 0; i < 20; i++) {
        float z = qrot(dice.q, faceN[i]).z;
        if (z > bz) { bz = z; best = i; }
    }
    return best;
}

static void update_dice(float dt) {
    if (!dice.active) return;
    dice.t += dt;
    float S = (float)(FW < FH ? FW : FH);
    float rad = S * 0.11f;
    if (dice.t < 1.5f) {
        dice.x += dice.vx * dt; dice.y += dice.vy * dt;
        float fr = fexp_neg(dt * 2.4f);
        dice.vx *= fr; dice.vy *= fr;
        if (dice.x < rad) { dice.x = rad; dice.vx = -dice.vx * 0.7f; dice.w.y += 8; }
        if (dice.x > FW - rad) { dice.x = FW - rad; dice.vx = -dice.vx * 0.7f; dice.w.y -= 8; }
        if (dice.y < rad + FH * 0.08f) { dice.y = rad + FH * 0.08f; dice.vy = -dice.vy * 0.7f; dice.w.x -= 8; }
        if (dice.y > FH - rad - FH * 0.12f && dice.vy > 0) { dice.vy = -dice.vy * 0.7f; dice.w.x += 8; }
        dice.w = mul(dice.w, fexp_neg(dt * 1.1f));
    } else {
        dice.x += dice.vx * dt; dice.y += dice.vy * dt;
        float fr = fexp_neg(dt * 6.f);
        dice.vx *= fr; dice.vy *= fr;
        dice.w = mul(dice.w, fexp_neg(dt * 7.f));
        // мягко укладываем верхнюю грань «лицом» к зрителю
        int top = dice_top();
        V3 n = qrot(dice.q, faceN[top]);
        V3 ax = cross(n, v3(0, 0, 1));
        float s = fsqrt(dot(ax, ax));
        if (s > 1e-5f) {
            float ang = fatan2(s, n.z);
            float k = fmin_(1.f, dt * 7.f);
            dice.q = qnorm(qmul(qaxis(mul(ax, 1.f / s), ang * k), dice.q));
        }
        // и поворачиваем вокруг оси взгляда, чтобы число читалось ровно
        V3 o0 = icoV[icoF[top][0]], o1 = icoV[icoF[top][1]], o2 = icoV[icoF[top][2]];
        V3 upv = qrot(dice.q, sub(o0, mul(add(add(o0, o1), o2), 1.f / 3)));
        float phi = fatan2(upv.x, upv.y);
        dice.q = qnorm(qmul(qaxis(v3(0, 0, 1), phi * fmin_(1.f, dt * 5.f)), dice.q));
        if (!dice.settled && dice.t > 2.3f) {
            dice.settled = true;
            dice.top = dice_top();
            dice.result = faceNum[dice.top];
            dice.event = 0;
            if (dice.attack && attackTarget >= 0) {
                Monster& m = mons[attackTarget];
                if (dice.result == 20) dice.event = 3;
                else if (dice.result == 1) dice.event = 4;
                else dice.event = dice.result >= 11 ? 1 : 2;
                if (dice.event == 1 || dice.event == 3) {
                    m.alive = false; m.death = 1;
                    burst(m.x, m.y, 1.0f, 0.75f, 0.3f, dice.event == 3 ? 90 : 50);
                    burst(m.x, m.y, 0.9f, 0.2f, 0.15f, 30);
                } else {
                    m.hit = 0.6f;
                }
                attackTarget = -1;
            }
        }
    }
    Quat wq = {0, dice.w.x, dice.w.y, dice.w.z};
    Quat dq = qmul(wq, dice.q);
    dice.q.w += 0.5f * dq.w * dt; dice.q.x += 0.5f * dq.x * dt;
    dice.q.y += 0.5f * dq.y * dt; dice.q.z += 0.5f * dq.z * dt;
    dice.q = qnorm(dice.q);
    if (dice.t > 4.6f) dice.scale = 1.f - smooth((dice.t - 4.6f) / 0.35f);
    if (dice.t > 5.0f) dice.active = false;
}

// ---------------------------------------------------------------- pixels

static inline float aces(float x) {
    float r = (x * (2.51f * x + 0.03f)) / (x * (2.43f * x + 0.59f) + 0.14f);
    return r > 1 ? 1 : (r < 0 ? 0 : r);
}
static inline u32 pack(float r, float g, float b) {
    u32 R = (u32)(aces(r) * 255.f + 0.5f), G = (u32)(aces(g) * 255.f + 0.5f), B = (u32)(aces(b) * 255.f + 0.5f);
    return R | (G << 8) | (B << 16) | 0xff000000u;
}
static inline u32 pack_raw(float r, float g, float b) {
    u32 R = (u32)(sat(r) * 255.f + 0.5f), G = (u32)(sat(g) * 255.f + 0.5f), B = (u32)(sat(b) * 255.f + 0.5f);
    return R | (G << 8) | (B << 16) | 0xff000000u;
}
static inline void blend_px(int x, int y, float r, float g, float b, float a) {
    if ((unsigned)x >= (unsigned)FW || (unsigned)y >= (unsigned)FH || a <= 0) return;
    u32* p = fb + y * FW + x;
    u32 c = *p;
    float R = (c & 255) / 255.f, G = ((c >> 8) & 255) / 255.f, B = ((c >> 16) & 255) / 255.f;
    a = sat(a);
    *p = pack_raw(R + (r - R) * a, G + (g - G) * a, B + (b - B) * a);
}
static inline void add_px(int x, int y, float r, float g, float b) {
    if ((unsigned)x >= (unsigned)FW || (unsigned)y >= (unsigned)FH) return;
    u32* p = fb + y * FW + x;
    u32 c = *p;
    float R = (c & 255) / 255.f, G = ((c >> 8) & 255) / 255.f, B = ((c >> 16) & 255) / 255.f;
    *p = pack_raw(R + r, G + g, B + b);
}

static inline void sample_lm(float wx, float wy, float* o) {
    float gx = wx * LS - 0.5f, gy = wy * LS - 0.5f;
    int ix = (int)ffloor(gx), iy = (int)ffloor(gy);
    float fx = gx - ix, fy = gy - iy;
    if (ix < 0) { ix = 0; fx = 0; }
    if (iy < 0) { iy = 0; fy = 0; }
    if (ix > LW - 2) { ix = LW - 2; fx = 1; }
    if (iy > LH - 2) { iy = LH - 2; fy = 1; }
    const float* c00 = lm + (iy * LW + ix) * 4;
    const float* c10 = c00 + 4;
    const float* c01 = c00 + LW * 4;
    const float* c11 = c01 + 4;
    float w00 = (1 - fx) * (1 - fy), w10 = fx * (1 - fy), w01 = (1 - fx) * fy, w11 = fx * fy;
    for (int k = 0; k < 4; k++) o[k] = c00[k] * w00 + c10[k] * w10 + c01[k] * w01 + c11[k] * w11;
}

static inline float sample_vis(float wx, float wy) {
    if (!fogOn) return 1;
    int x = (int)(wx * LS), y = (int)(wy * LS);
    if (x < 0 || y < 0 || x >= LW || y >= LH) return 0;
    return visA[y * LW + x];
}
static inline float sample_expl(float wx, float wy) {
    if (!fogOn) return 1;
    int x = (int)(wx * LS), y = (int)(wy * LS);
    if (x < 0 || y < 0 || x >= LW || y >= LH) return 0;
    return expl[y * LW + x];
}

// ---------------------------------------------------------------- lighting

static float flicker(const Light& L) {
    float t = timeNow + L.phase;
    if (L.kind == L_CRYSTAL) return 0.8f + 0.2f * fsin(t * 1.7f);
    return 0.82f + 0.1f * fsin(t * 9.1f) + 0.06f * fsin(t * 23.7f + 1.3f) + 0.04f * fsin(t * 41.f);
}

static void compute_lighting() {
    const int N = LW * LH;
    for (int i = 0; i < N * 4; i++) lm[i] = 0;
    for (int l = 0; l < nlight; l++) {
        const Light& L = lights[l];
        float f = flicker(L);
        float r = L.r * f, g = L.g * f, b = L.b * f;
        for (u32 k = L.start; k < L.start + L.count; k++) {
            float* c = lm + pool[k].idx * 4;
            float w = pool[k].w;
            c[0] += r * w; c[1] += g * w; c[2] += b * w;
        }
    }
    // видимость героя + фонарь героя за один проход лучей
    for (int i = 0; i < N; i++) visA[i] = 0;
    const float RV = 11.5f, RH = 6.5f;
    int R = (int)(RV * LS) + 1;
    int cx = (int)(heroX * LS), cy = (int)(heroY * LS);
    float lf = 0.92f + 0.05f * fsin(timeNow * 7.3f) + 0.03f * fsin(timeNow * 17.f);
    for (int y = cy - R; y <= cy + R; y++) {
        if (y < 0 || y >= LH) continue;
        for (int x = cx - R; x <= cx + R; x++) {
            if (x < 0 || x >= LW) continue;
            float wx = (x + 0.5f) / LS, wy = (y + 0.5f) / LS;
            float dx = wx - heroX, dy = wy - heroY;
            float d2 = dx * dx + dy * dy;
            if (d2 >= RV * RV) continue;
            if (!los(heroX, heroY, wx, wy)) continue;
            float d = fsqrt(d2);
            int i = y * LW + x;
            visA[i] = smooth((RV - d) / 2.5f);
            if (lanternOn && d < RH) {
                float a = 1.f - d / RH;
                float w = (a * a + 0.3f * a * a * a * a) * 1.25f * lf;
                lm[i * 4 + 0] += 1.0f * w; lm[i * 4 + 1] += 0.84f * w; lm[i * 4 + 2] += 0.6f * w;
            }
        }
    }
    for (int i = 0; i < N; i++) {
        float* c = lm + i * 4;
        float v = 1, m = 0;
        if (fogOn) {
            v = visA[i];
            if (v > expl[i]) expl[i] = v;
            m = expl[i] * (1 - v);
        }
        c[0] = (c[0] + 0.035f) * v;
        c[1] = (c[1] + 0.035f) * v;
        c[2] = (c[2] + 0.05f) * v;
        c[3] = m;
    }
}

// ---------------------------------------------------------------- scene shading

static inline void shade_table(float wx, float wy, float& r, float& g, float& b) {
    // деревянный стол вокруг карты и золотая рамка
    float ex = fmax_(fmax_(-wx, wx - MW), fmax_(-wy, wy - MH));
    if (ex < 0.35f) {
        float k = 0.75f + 0.5f * (1.f - ex / 0.35f) * (ex < 0.12f ? 1.f : 0.4f);
        r = 0.95f * k; g = 0.68f * k; b = 0.3f * k;
        float L = 0.7f;
        r *= L; g *= L; b *= L;
        return;
    }
    float plank = ffloor(wy / 2.5f);
    float py = wy / 2.5f - plank;
    float ph = hashf((int)plank, 17);
    float grain = fsin(wx * 0.9f + ph * 40.f + 2.5f * fsin(wy * 1.3f + wx * 0.07f + ph * 9)) * 0.5f + 0.5f;
    float n = (hash2((int)(wx * 10), (int)(wy * 40)) & 255) / 255.f;
    float base = 0.7f + 0.2f * grain + 0.1f * n + 0.15f * ph;
    if (py < 0.03f || py > 0.97f) base *= 0.45f;
    r = 0.36f * base; g = 0.2f * base; b = 0.1f * base;
    float cx = wx - MW * 0.5f, cy = wy - MH * 0.5f;
    float d = fsqrt(cx * cx + cy * cy) / (MW * 0.9f);
    float L = 0.85f * sat(1.2f - d) + 0.12f;
    r *= L; g *= L * 0.95f; b *= L * 0.85f;
}

static inline void albedo(int tx, int ty, float fx, float fy, float wx, float wy, float& ar, float& ag, float& ab) {
    u8 t = tiles[ty][tx];
    u8 nm = nmask[ty][tx];
    float hv = tileHash[ty][tx];
    if (t == FLOOR) {
        int ix = (int)(wx * 16), iy = (int)(wy * 16);
        float n = (hash2(ix, iy) & 255) * (1.f / 255.f);
        float n2 = (hash2(ix >> 2, iy >> 2) & 255) * (1.f / 255.f);
        float base = 0.72f + 0.3f * hv + 0.1f * n + 0.08f * n2;
        ar = 0.47f * base; ag = 0.43f * base; ab = 0.38f * base;
        if (hv > 0.86f) { ag *= 1.18f; ar *= 0.88f; ab *= 0.85f; }
        float e = fmin_(fmin_(fx, 1 - fx), fmin_(fy, 1 - fy));
        if (e < 0.05f) { float k = 0.42f + 0.58f * (e / 0.05f); ar *= k; ag *= k; ab *= k; }
        else if (e < 0.09f) { float k = 1.08f; ar *= k; ag *= k; ab *= k; }
        float ao = 1;
        if (nm & 1) ao *= 0.35f + 0.65f * smooth(fx * 2.6f);
        if (nm & 2) ao *= 0.35f + 0.65f * smooth((1 - fx) * 2.6f);
        if (nm & 4) ao *= 0.35f + 0.65f * smooth(fy * 2.6f);
        if (nm & 8) ao *= 0.35f + 0.65f * smooth((1 - fy) * 2.6f);
        ar *= ao; ag *= ao; ab *= ao;
    } else if (nm & 16) {
        // кирпичная стена с «лицевой» стороной к югу — псевдообъём
        bool face = (nm & 8) && fy > 0.58f;
        float by = wy * (face ? 5.f : 3.f);
        int row = (int)ffloor(by);
        float bx = wx * 2.f + (row & 1) * 0.5f;
        int col = (int)ffloor(bx);
        float fbx = bx - col, fby = by - row;
        float h = hashf(col, row);
        float n = (hash2((int)(wx * 16), (int)(wy * 16)) & 255) * (1.f / 255.f);
        float base = 0.62f + 0.25f * h + 0.1f * n;
        ar = 0.55f * base; ag = 0.47f * base; ab = 0.42f * base;
        if (fbx < 0.06f || fby < 0.1f) { ar *= 0.45f; ag *= 0.45f; ab *= 0.45f; }
        if (face) {
            float k = 0.55f - 0.25f * (fy - 0.58f) / 0.42f;
            ar *= k; ag *= k; ab *= k;
        } else {
            float k = 1.15f;
            if ((nm & 8) && fy > 0.5f) k = 1.35f;  // светлая кромка
            ar *= k; ag *= k; ab *= k;
        }
    } else {
        float n = (hash2((int)(wx * 8), (int)(wy * 8)) & 255) * (1.f / 255.f);
        ar = 0.06f + 0.03f * n; ag = 0.055f + 0.025f * n; ab = 0.06f + 0.03f * n;
    }
}

static void render_scene() {
    float inv = 1.f / zoom;
    float halfW = FW * 0.5f, halfH = FH * 0.5f;
    float o[4];
    for (int py = 0; py < FH; py++) {
        float wy = camY + (py + 0.5f - halfH) * inv;
        float vy = (py + 0.5f) / FH - 0.5f;
        u32* row = fb + py * FW;
        float wx = camX + (0.5f - halfW) * inv;
        for (int px = 0; px < FW; px++, wx += inv) {
            float r, g, b;
            if (wx < 0 || wy < 0 || wx >= MW || wy >= MH) {
                shade_table(wx, wy, r, g, b);
            } else {
                int tx = (int)wx, ty = (int)wy;
                float ar, ag, ab;
                albedo(tx, ty, wx - tx, wy - ty, wx, wy, ar, ag, ab);
                sample_lm(wx, wy, o);
                float gray = (ar + ag + ab) * 0.333f * o[3];
                r = ar * o[0] * 1.6f + gray * 0.10f;
                g = ag * o[1] * 1.6f + gray * 0.12f;
                b = ab * o[2] * 1.6f + gray * 0.20f;
            }
            float vx = (px + 0.5f) / FW - 0.5f;
            float vig = 1.f - 0.9f * (vx * vx + vy * vy);
            row[px] = pack(r * vig * 1.15f, g * vig * 1.15f, b * vig * 1.15f);
        }
    }
}

// ---------------------------------------------------------------- sprites

static inline float sx_of(float wx) { return (wx - camX) * zoom + FW * 0.5f; }
static inline float sy_of(float wy) { return (wy - camY) * zoom + FH * 0.5f; }

static void glow(float sx, float sy, float rad, float r, float g, float b, float inten) {
    int x0 = (int)(sx - rad), x1 = (int)(sx + rad) + 1, y0 = (int)(sy - rad), y1 = (int)(sy + rad) + 1;
    if (x1 < 0 || y1 < 0 || x0 >= FW || y0 >= FH) return;
    float ir = 1.f / rad;
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) {
            float dx = (x + 0.5f - sx) * ir, dy = (y + 0.5f - sy) * ir;
            float d2 = dx * dx + dy * dy;
            if (d2 >= 1) continue;
            float a = (1 - d2); a *= a * inten;
            add_px(x, y, r * a, g * a, b * a);
        }
}

static void shadow_disc(float sx, float sy, float rad, float alpha) {
    int x0 = (int)(sx - rad), x1 = (int)(sx + rad) + 1, y0 = (int)(sy - rad), y1 = (int)(sy + rad) + 1;
    if (x1 < 0 || y1 < 0 || x0 >= FW || y0 >= FH) return;
    float ir = 1.f / rad;
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) {
            float dx = (x + 0.5f - sx) * ir, dy = (y + 0.5f - sy) * ir;
            float d2 = dx * dx + dy * dy;
            if (d2 >= 1) continue;
            blend_px(x, y, 0, 0, 0, alpha * (1 - d2));
        }
}

// фишка-миниатюра: металлический ободок, выпуклое тело, эмблема
static void draw_token(float wx, float wy, float radT, float br, float bg, float bb,
                       float rr, float rg, float rb, int emblem, float flash, float alpha) {
    float sx = sx_of(wx), sy = sy_of(wy), rad = radT * zoom;
    if (sx + rad * 1.5f < 0 || sy + rad * 1.5f < 0 || sx - rad * 1.5f > FW || sy - rad * 1.5f > FH) return;
    float o[4];
    sample_lm(wx, wy, o);
    float lr = 0.42f + o[0] * 0.9f, lg = 0.42f + o[1] * 0.9f, lb = 0.45f + o[2] * 0.9f;
    shadow_disc(sx + rad * 0.18f, sy + rad * 0.28f, rad * 1.12f, 0.55f * alpha);
    int x0 = (int)(sx - rad) - 1, x1 = (int)(sx + rad) + 1, y0 = (int)(sy - rad) - 1, y1 = (int)(sy + rad) + 1;
    float ir = 1.f / rad;
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) {
            float dx = (x + 0.5f - sx) * ir, dy = (y + 0.5f - sy) * ir;
            float d = fsqrt(dx * dx + dy * dy);
            if (d >= 1.f) continue;
            float edge = sat((1.f - d) * rad);
            float r, g, b;
            bool emissive = false;
            if (d > 0.78f) {
                float t = (d - 0.78f) / 0.22f;
                float sh = 0.55f + 0.9f * sat(0.5f - dx * 0.5f - dy * 0.7f) * (1.f - fabs_(t - 0.4f));
                r = rr * sh; g = rg * sh; b = rb * sh;
            } else {
                float sh = 0.7f + 0.45f * (-dx * 0.55f - dy * 0.75f) - 0.25f * d * d;
                r = br * sh; g = bg * sh; b = bb * sh;
                if (emblem == 0) {  // меч героя
                    bool blade = fabs_(dx) < 0.075f && dy > -0.55f && dy < 0.32f;
                    bool guard = fabs_(dy - 0.18f) < 0.06f && fabs_(dx) < 0.3f;
                    bool hilt = fabs_(dx) < 0.06f && dy >= 0.32f && dy < 0.5f;
                    if (blade) { float s = 0.85f + 0.4f * (dx < 0); r = 0.85f * s; g = 0.9f * s; b = 1.f * s; }
                    if (guard || hilt) { r = 1.f; g = 0.78f; b = 0.3f; }
                } else if (emblem == 1) {  // гоблин: светящиеся глаза
                    float ex = fabs_(dx) - 0.26f, ey = dy + 0.08f;
                    if (ex * ex + ey * ey < 0.016f) { r = 1.f; g = 0.25f; b = 0.1f; emissive = true; }
                    if (fabs_(dy - 0.3f) < 0.04f && fabs_(dx) < 0.28f) { r *= 0.3f; g *= 0.3f; b *= 0.3f; }
                } else if (emblem == 2) {  // скелет: глазницы и зубы
                    float ex = fabs_(dx) - 0.25f, ey = dy + 0.05f;
                    float e2 = ex * ex + ey * ey;
                    if (e2 < 0.025f) {
                        if (e2 < 0.006f) { r = 0.75f; g = 0.4f; b = 1.f; emissive = true; }
                        else { r = 0.05f; g = 0.04f; b = 0.06f; }
                    }
                    if (dy > 0.25f && dy < 0.42f && fabs_(dx) < 0.3f) {
                        float tooth = dx * 12.f - ffloor(dx * 12.f);
                        if (tooth < 0.25f) { r *= 0.3f; g *= 0.3f; b *= 0.3f; }
                    }
                }
            }
            if (!emissive) { r *= lr; g *= lg; b *= lb; }
            if (flash > 0) { r = mix(r, 1, flash); g = mix(g, 0.9f, flash); b = mix(b, 0.85f, flash); }
            blend_px(x, y, r, g, b, edge * alpha);
        }
}

static void ring(float sx, float sy, float rad, float width, float r, float g, float b, float alpha, float dash, float rot) {
    int R = (int)(rad + width) + 2;
    for (int y = (int)sy - R; y <= (int)sy + R; y++)
        for (int x = (int)sx - R; x <= (int)sx + R; x++) {
            float dx = x + 0.5f - sx, dy = y + 0.5f - sy;
            float d = fsqrt(dx * dx + dy * dy);
            float k = 1.f - fabs_(d - rad) / width;
            if (k <= 0) continue;
            if (dash > 0) {
                float a = fatan2(dy, dx) + rot;
                float s = fsin(a * dash);
                if (s < 0) continue;
                k *= sat(s * 4);
            }
            add_px(x, y, r * k * alpha, g * k * alpha, b * k * alpha);
        }
}

static void draw_sprites() {
    // факелы и кристаллы
    for (int i = 0; i < nlight; i++) {
        const Light& L = lights[i];
        if (fogOn && sample_expl(L.x, L.y) < 0.2f) continue;
        float sx = sx_of(L.x), sy = sy_of(L.y);
        float f = flicker(L);
        if (L.kind == L_TORCH) {
            bool top = !solid((int)L.x, (int)(L.y - 0.5f)) ? false : (L.y - ffloor(L.y)) < 0.5f;
            float fy = top ? sy - zoom * 0.2f : sy + zoom * 0.05f;
            // кронштейн
            for (int y = 0; y < (int)(zoom * 0.22f); y++)
                for (int x = -(int)(zoom * 0.04f); x <= (int)(zoom * 0.04f); x++)
                    blend_px((int)sx + x, (int)(fy + zoom * 0.04f) + (top ? -y : y) * 0 + y, 0.18f, 0.12f, 0.08f, 0.9f);
            glow(sx, fy, zoom * 0.95f * f, 0.55f, 0.24f, 0.06f, 0.35f);
            glow(sx, fy, zoom * 0.28f * f, 1.f, 0.6f, 0.2f, 0.9f);
            glow(sx, fy, zoom * 0.12f, 1.f, 0.95f, 0.7f, 1.0f);
        } else {
            float p = 0.5f + 0.5f * fsin(timeNow * 1.7f + L.phase);
            glow(sx, sy, zoom * (0.9f + 0.2f * p), L.r * 0.4f, L.g * 0.4f, L.b * 0.4f, 0.6f);
            // кристалл-ромб
            float h = zoom * 0.42f, w = zoom * 0.22f;
            for (int y = (int)(sy - h); y <= (int)(sy + h * 0.6f); y++)
                for (int x = (int)(sx - w); x <= (int)(sx + w); x++) {
                    float dx = fabs_(x + 0.5f - sx) / w;
                    float dy = (y + 0.5f - sy);
                    float lim = dy < 0 ? 1.f + dy / h : 1.f - dy / (h * 0.6f);
                    if (dx > lim) continue;
                    float s = (x + 0.5f < sx) ? 1.0f : 0.65f;
                    blend_px(x, y, mix(L.r, 1, 0.35f) * s, mix(L.g, 1, 0.35f) * s, mix(L.b, 1, 0.35f) * s, 0.92f);
                }
            glow(sx, sy - h * 0.3f, zoom * 0.18f, 1, 1, 1, 0.8f * p);
        }
    }

    // путь героя
    for (int i = pathIdx; i < pathLen; i++) {
        float px = (path[i] % MW) + 0.5f, py = (path[i] / MW) + 0.5f;
        float pulse = 0.6f + 0.4f * fsin(timeNow * 6.f - i * 0.6f);
        glow(sx_of(px), sy_of(py), zoom * 0.14f, 1.f, 0.8f, 0.35f, 0.9f * pulse);
    }
    if (pathIdx < pathLen) {
        int last = path[pathLen - 1];
        float px = (last % MW) + 0.5f, py = (last / MW) + 0.5f;
        float pr = zoom * (0.3f + 0.06f * fsin(timeNow * 5.f));
        ring(sx_of(px), sy_of(py), pr, zoom * 0.05f, 1.f, 0.85f, 0.4f, 0.9f, 0, 0);
    }

    // рябь от касаний
    for (int i = 0; i < nrip; i++) {
        Ripple& R = ripples[i];
        float k = R.t / 0.6f;
        float a = 1.f - k;
        ring(sx_of(R.x), sy_of(R.y), zoom * (0.15f + 0.5f * k), zoom * 0.05f,
             R.bad ? 1.f : 0.9f, R.bad ? 0.3f : 0.8f, R.bad ? 0.25f : 0.5f, a, 0, 0);
    }

    // монстры
    for (int i = 0; i < nmon; i++) {
        Monster& m = mons[i];
        if (!m.alive && m.death <= 0) continue;
        float v = sample_vis(m.x, m.y);
        if (v < 0.05f) continue;
        float bob = fsin(timeNow * 2.2f + m.phase) * 0.03f;
        float a = v * (m.alive ? 1.f : m.death);
        if (attackTarget == i || pendingAttack == i)
            ring(sx_of(m.x), sy_of(m.y), zoom * 0.48f, zoom * 0.05f, 1.f, 0.25f, 0.2f, 0.9f, 6, timeNow * 2.f);
        if (m.type == M_GOBLIN)
            draw_token(m.x, m.y + bob, 0.37f, 0.36f, 0.5f, 0.25f, 0.75f, 0.22f, 0.18f, 1, m.hit, a);
        else
            draw_token(m.x, m.y + bob, 0.38f, 0.86f, 0.83f, 0.74f, 0.55f, 0.3f, 0.75f, 2, m.hit, a);
    }

    // герой
    float hsx = sx_of(heroX), hsy = sy_of(heroY);
    ring(hsx, hsy, zoom * 0.5f, zoom * 0.04f, 1.f, 0.8f, 0.35f, 0.85f, 8, timeNow * 0.8f);
    float bob = pathIdx < pathLen ? fabs_(fsin(heroPhase * 2.f)) * 0.05f : 0;
    draw_token(heroX, heroY - bob, 0.38f, 0.2f, 0.32f, 0.72f, 1.f, 0.78f, 0.32f, 0, 0, 1);

    // частицы
    for (int i = 0; i < nparts; i++) {
        Particle& p = parts[i];
        if (sample_vis(p.x, p.y) < 0.05f) continue;
        float k = p.life / p.max;
        glow(sx_of(p.x), sy_of(p.y), fmax_(1.5f, p.size * zoom), p.r, p.g, p.b, k * 1.2f);
    }
}

// ---------------------------------------------------------------- dice raster

static const u8 FONT[10][5] = {
    {7, 5, 5, 5, 7}, {2, 6, 2, 2, 7}, {7, 1, 7, 4, 7}, {7, 1, 7, 1, 7}, {5, 5, 7, 1, 1},
    {7, 4, 7, 1, 7}, {7, 4, 7, 5, 7}, {7, 1, 1, 2, 2}, {7, 5, 7, 5, 7}, {7, 5, 7, 1, 7}};

static bool glyph_hit(int num, float u, float v, float h) {
    // u вправо, v вверх; ячейка шрифта = h/5
    float cell = h / 5.f;
    int digits[2]; int nd = 0;
    if (num >= 10) { digits[nd++] = num / 10; digits[nd++] = num % 10; }
    else digits[nd++] = num;
    float totalW = nd * 3 * cell + (nd - 1) * cell;
    float gx = (u + totalW * 0.5f) / cell, gy = (h * 0.5f - v) / cell;
    if (gx < 0 || gy < 0 || gy >= 5) return false;
    int ix = (int)gx, iy = (int)gy;
    // подчёркивание у 6 и 9, как на настоящих костях
    if ((num == 6 || num == 9) && gy >= 5) return false;
    int d = ix / 4, cx = ix % 4;
    if (d >= nd || cx == 3) return false;
    return (FONT[digits[d]][iy] >> (2 - cx)) & 1;
}

static void draw_dice() {
    if (!dice.active) return;
    float S = (float)(FW < FH ? FW : FH) * 0.11f * dice.scale;
    if (S < 1) return;
    float lift = dice.t < 1.5f ? 0.25f + 0.15f * fabs_(fsin(dice.t * 7.f)) * (1.5f - dice.t) : 0.18f;
    shadow_disc(dice.x + S * lift, dice.y + S * (lift + 0.25f), S * 1.05f, 0.5f);
    if (dice.settled) {
        float c = dice.result == 20 ? 1.f : 0.f, f = dice.result == 1 ? 1.f : 0.f;
        float p = 0.6f + 0.4f * fsin(timeNow * 5.f);
        if (c > 0) glow(dice.x, dice.y, S * 2.4f, 0.6f * p, 0.45f * p, 0.1f * p, 0.7f);
        if (f > 0) glow(dice.x, dice.y, S * 2.4f, 0.6f * p, 0.05f, 0.05f, 0.6f);
    }
    V3 sv[12];
    for (int i = 0; i < 12; i++) sv[i] = qrot(dice.q, icoV[i]);
    V3 L = norm(v3(-0.5f, 0.75f, 0.8f));
    V3 H = norm(add(L, v3(0, 0, 1)));
    for (int f = 0; f < 20; f++) {
        V3 n = qrot(dice.q, faceN[f]);
        if (n.z <= 0.0f) continue;
        V3 a = sv[icoF[f][0]], b = sv[icoF[f][1]], c = sv[icoF[f][2]];
        float ax = dice.x + a.x * S, ay = dice.y - a.y * S;
        float bx = dice.x + b.x * S, by = dice.y - b.y * S;
        float cx = dice.x + c.x * S, cy = dice.y - c.y * S;
        float area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        if (fabs_(area) < 1e-3f) continue;
        float ia = 1.f / area;
        int x0 = (int)fmin_(ax, fmin_(bx, cx)), x1 = (int)fmax_(ax, fmax_(bx, cx)) + 1;
        int y0 = (int)fmin_(ay, fmin_(by, cy)), y1 = (int)fmax_(ay, fmax_(by, cy)) + 1;
        float diff = fmax_(0, dot(n, L));
        float sp = fmax_(0, dot(n, H));
        sp *= sp; sp *= sp; sp *= sp; sp *= sp; sp *= sp;
        // локальная система грани для цифры (в координатах объекта)
        V3 oa = icoV[icoF[f][0]], ob = icoV[icoF[f][1]], oc = icoV[icoF[f][2]];
        V3 cen = mul(add(add(oa, ob), oc), 1.f / 3);
        V3 up = norm(sub(oa, cen));
        V3 rt = norm(cross(up, faceN[f]));
        int num = faceNum[f];
        bool topFace = dice.settled && f == dice.top;
        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++) {
                float px = x + 0.5f, py = y + 0.5f;
                float w0 = ((bx - px) * (cy - py) - (by - py) * (cx - px)) * ia;
                float w1 = ((cx - px) * (ay - py) - (cy - py) * (ax - px)) * ia;
                float w2 = 1.f - w0 - w1;
                if (w0 < 0 || w1 < 0 || w2 < 0) continue;
                float edge = fmin_(w0, fmin_(w1, w2));
                V3 P = add(add(mul(oa, w0), mul(ob, w1)), mul(oc, w2));
                V3 d = sub(P, cen);
                float u = dot(d, rt), v = dot(d, up);
                bool g = glyph_hit(num, u, v, 0.27f);
                bool ul = (num == 6 || num == 9) && v < -0.165f && v > -0.2f && fabs_(u) < 0.1f;
                float r, gg, bb;
                if (g || ul) {
                    float k = 0.75f + 0.5f * diff;
                    r = 1.f * k; gg = 0.8f * k; bb = 0.35f * k;
                    if (topFace) { r = 1.f; gg = 0.92f; bb = 0.55f; }
                } else {
                    float k = 0.18f + 0.85f * diff;
                    r = 0.62f * k + 0.05f; gg = 0.06f * k + 0.01f; bb = 0.12f * k + 0.03f;
                    // внутреннее «свечение» полупрозрачного камня
                    float inner = 1.f - edge * 2.2f;
                    r += 0.12f * sat(inner) * n.z;
                }
                if (edge < 0.035f) { float k = 1.f + 0.6f * (1.f - edge / 0.035f); r *= k; gg *= k; bb *= k; }
                r += sp * 0.9f; gg += sp * 0.85f; bb += sp * 0.8f;
                float aa = sat(edge * S * 1.2f + 0.3f);
                blend_px(x, y, r, gg, bb, aa);
            }
    }
}

// ---------------------------------------------------------------- update

static void update(float dt) {
    timeNow += dt;
    // движение героя по пути
    if (pathIdx < pathLen) {
        int n = path[pathIdx];
        float tx = (n % MW) + 0.5f, ty = (n / MW) + 0.5f;
        float dx = tx - heroX, dy = ty - heroY;
        float d = fsqrt(dx * dx + dy * dy);
        float step = 4.6f * dt;
        heroPhase += dt * 9.f;
        if (d <= step) { heroX = tx; heroY = ty; pathIdx++; }
        else { heroX += dx / d * step; heroY += dy / d * step; }
        if (pathIdx >= pathLen && pendingAttack >= 0) {
            attackTarget = pendingAttack; pendingAttack = -1;
            roll_dice(true);
        }
    }
    if (follow) {
        float k = 1.f - fexp_neg(dt * 4.f);
        camX += (heroX - camX) * k;
        camY += (heroY - camY) * k;
    }
    for (int i = 0; i < nmon; i++) {
        Monster& m = mons[i];
        if (m.hit > 0) m.hit = fmax_(0, m.hit - dt * 1.5f);
        if (!m.alive && m.death > 0) m.death = fmax_(0, m.death - dt * 1.4f);
    }
    // эмиттеры
    for (int i = 0; i < nlight; i++) {
        Light& L = lights[i];
        L.spawn += dt * (L.kind == L_TORCH ? 7.f : 3.f);
        while (L.spawn > 1) {
            L.spawn -= 1;
            if (L.kind == L_TORCH) {
                bool top = (L.y - ffloor(L.y)) < 0.5f;
                spawn(L.x + (rndf() - 0.5f) * 0.12f, L.y + (top ? -0.2f : 0.05f), (rndf() - 0.5f) * 0.3f,
                      -0.5f - rndf() * 0.6f, 0.7f + rndf() * 0.8f, 1.f, 0.45f + rndf() * 0.3f, 0.1f, 0.035f + rndf() * 0.03f);
            } else {
                float a = rndf() * 6.28f;
                spawn(L.x + fcos(a) * 0.3f, L.y + fsin(a) * 0.3f, 0, -0.35f - rndf() * 0.3f, 1.2f + rndf(),
                      L.r, L.g, L.b, 0.03f + rndf() * 0.03f);
            }
        }
    }
    for (int i = 0; i < nparts;) {
        Particle& p = parts[i];
        p.life -= dt;
        if (p.life <= 0) { parts[i] = parts[--nparts]; continue; }
        p.vx += fsin(timeNow * 3.f + i) * dt * 0.4f;
        p.x += p.vx * dt; p.y += p.vy * dt;
        float fr = fexp_neg(dt * 1.5f);
        p.vx *= fr; p.vy *= fr;
        i++;
    }
    for (int i = 0; i < nrip;) {
        ripples[i].t += dt;
        if (ripples[i].t > 0.6f) { ripples[i] = ripples[--nrip]; continue; }
        i++;
    }
    update_dice(dt);
}

// ---------------------------------------------------------------- exports

EXPORT(init) void init(u32 seed) {
    init_ico();
    generate(seed);
}

EXPORT(resize) u32* resize(int w, int h) {
    if (w > MAXW) w = MAXW;
    if (h > MAXH) h = MAXH;
    if (w * h > MAXW * MAXH / 2) h = MAXW * MAXH / 2 / w;
    float oldMin = (float)(FW < FH ? FW : FH);
    FW = w; FH = h;
    float newMin = (float)(FW < FH ? FW : FH);
    if (oldMin <= 0) zoom = newMin / 9.5f;
    else zoom *= newMin / oldMin;
    return fb;
}

EXPORT(frame) void frame(float dt) {
    if (dt > 0.1f) dt = 0.1f;
    update(dt);
    compute_lighting();
    render_scene();
    draw_sprites();
    draw_dice();
}

EXPORT(tap) void tap(float x, float y) {
    float wx = camX + (x - FW * 0.5f) / zoom, wy = camY + (y - FH * 0.5f) / zoom;
    int tx = (int)ffloor(wx), ty = (int)ffloor(wy);
    if (nrip >= 8) nrip = 7;
    Ripple& R = ripples[nrip++];
    R.x = wx; R.y = wy; R.t = 0; R.bad = false;
    int hx = (int)heroX, hy = (int)heroY;
    // вне клетки? проверим, не монстр ли это
    for (int i = 0; i < nmon; i++) {
        Monster& m = mons[i];
        if (!m.alive || (int)m.x != tx || (int)m.y != ty) continue;
        if (sample_vis(m.x, m.y) < 0.3f) break;
        if (dice.active && !dice.settled) return;
        follow = true;
        int ddx = tx - hx, ddy = ty - hy;
        if (ddx >= -1 && ddx <= 1 && ddy >= -1 && ddy <= 1) {
            pathLen = pathIdx = 0;
            attackTarget = i; pendingAttack = -1;
            roll_dice(true);
            return;
        }
        if (astar(hx, hy, tx, ty) && pathLen > 0) {
            pathLen--;  // встаём рядом, а не на врага
            pendingAttack = i;
            if (pathLen == 0) { pendingAttack = -1; attackTarget = i; roll_dice(true); }
        } else R.bad = true;
        return;
    }
    pendingAttack = -1;
    if (solid(tx, ty) || (fogOn && sample_expl(wx, wy) < 0.3f) || !astar(hx, hy, tx, ty)) {
        R.bad = true;
        return;
    }
    follow = true;
}

EXPORT(pan) void pan(float dx, float dy) {
    camX -= dx / zoom; camY -= dy / zoom;
    if (camX < -4) camX = -4;
    if (camY < -4) camY = -4;
    if (camX > MW + 4) camX = MW + 4;
    if (camY > MH + 4) camY = MH + 4;
    follow = false;
}

EXPORT(zoom_at) void zoom_at(float f, float x, float y) {
    float wx = camX + (x - FW * 0.5f) / zoom, wy = camY + (y - FH * 0.5f) / zoom;
    float mn = (float)(FW < FH ? FW : FH);
    float nz = zoom * f;
    if (nz < mn / 40.f) nz = mn / 40.f;
    if (nz > mn / 3.5f) nz = mn / 3.5f;
    zoom = nz;
    camX = wx - (x - FW * 0.5f) / zoom;
    camY = wy - (y - FH * 0.5f) / zoom;
}

EXPORT(roll) void roll() {
    if (dice.active && !dice.settled) return;
    attackTarget = -1;
    roll_dice(false);
}
EXPORT(dice_result) int dice_result() {
    if (dice.active && dice.settled && !dice.reported) { dice.reported = 1; return dice.result; }
    return -1;
}
EXPORT(dice_event) int dice_event() { return dice.event; }
EXPORT(regenerate) void regenerate(u32 seed) { generate(seed); }
EXPORT(set_fog) void set_fog(int on) { fogOn = on != 0; }
EXPORT(set_lantern) void set_lantern(int on) { lanternOn = on != 0; }
EXPORT(recenter) void recenter() { follow = true; }
EXPORT(stat_lights) int stat_lights() { return nlight + (lanternOn ? 1 : 0); }
EXPORT(stat_splats) int stat_splats() { return (int)poolN; }
EXPORT(stat_particles) int stat_particles() { return nparts; }
EXPORT(stat_rooms) int stat_rooms() { return nroom; }
EXPORT(monsters_left) int monsters_left() {
    int n = 0;
    for (int i = 0; i < nmon; i++) n += mons[i].alive;
    return n;
}
EXPORT(monsters_total) int monsters_total() { return nmon; }
