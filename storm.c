#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int winWidth = 1920;
int winHeight = 1080;
#define TOPBAR 0
#define GRID_SCALE 4
#define PADDING_COST 50.0f
#define STORM_THRESHOLD 20.0f
#define VELOCITY_SAMPLES 5

// --- Map Boundary Constants (User Provided) ---
#define MAP_NORTH 85.14f
#define MAP_SOUTH -31.78f
#define MAP_WEST -16.08f
#define MAP_EAST 179.33f

typedef struct {
  float x, y;
  int valid;
  float alpha;
} Point;
typedef struct {
  int r, c;
} GridPos;
typedef struct {
  float windSpeed;  // km/h
  float windDir;    // degrees
  float waveHeight; // meters
} WeatherCell;

typedef struct Node {
  GridPos pos;
  float g, h, f;
  struct Node *parent;
} Node;

typedef struct {
  Node **nodes;
  int size;
} MinHeap;

// --- A* Heap Functions ---
void pushHeap(MinHeap *heap, Node *node) {
  int i = heap->size++;
  while (i > 0) {
    int p = (i - 1) / 2;
    if (heap->nodes[p]->f <= node->f)
      break;
    heap->nodes[i] = heap->nodes[p];
    i = p;
  }
  heap->nodes[i] = node;
}

Node *popHeap(MinHeap *heap) {
  if (heap->size == 0)
    return NULL;
  Node *res = heap->nodes[0];
  Node *last = heap->nodes[--heap->size];
  int i = 0;
  while (i * 2 + 1 < heap->size) {
    int child = i * 2 + 1;
    if (child + 1 < heap->size &&
        heap->nodes[child + 1]->f < heap->nodes[child]->f)
      child++;
    if (last->f <= heap->nodes[child]->f)
      break;
    heap->nodes[i] = heap->nodes[child];
    i = child;
  }
  heap->nodes[i] = last;
  return res;
}

// --- Globals ---
float zoom = 1.0f, targetZoom = 1.0f;
float camX = 0, camY = 0;
float velX = 0.0f, velY = 0.0f;
float friction = 0.94f;
float frameVelX[VELOCITY_SAMPLES] = {0};
float frameVelY[VELOCITY_SAMPLES] = {0};
int velIdx = 0;
float zoomWorldX = 0, zoomWorldY = 0; // World position to zoom towards
int zoomMouseX = 0, zoomMouseY = 0;   // Screen position of mouse during zoom
int showStorms = 1;                   // Toggle storm graphics on/off
int showWindParticles = 1;            // Toggle wind particles
int showWaveHeights = 1;              // Toggle wave gradient
float loadingRotation = 0.0f;         // For route analysis spinner
float pathDrawProgress = 0.0f;        // 0.0 to 1.0 for write-on effect
float spinnerTimer = 0.0f;            // Duration for loading spinner
int pendingVerification = 0;          // 1 if waiting for smart-route second pass
int smartRouteStage = 0;              // 0=None, 1=API Calling, 2=Optimizing Path
float smartRouteAnim = 0.0f;          // Dialogue box fade animation
int mouseX = 0, mouseY = 0;           // Mouse tracking for tooltip
int showJourneyPanel = 0;             // Journey planning sidebar toggle
float journeyPanelAnim = 0.0f;        // Side panel sliding animation (0 to 1)
char startCoordTxt[64] = "Not Set", endCoordTxt[64] = "Not Set";
char depDateTxt[32] = "2026-03-06 12:00"; // Default departure date and time
int pathHoverIndex = -1;              // Closest index on finalPath for interactive cursor
float pathDistances[4096];            // Cumulative distance along the path for timestamp calculation
float pathHoverT = 0.0f;              // Interpolation factor between segments
int isHoveringPath = 0;               // Hover state for tooltip
float hoverWind = 0, hoverWave = 0, hoverDir = 0;

// Threading for Non-Blocking API
int volatile weatherThreadActive = 0;
SDL_Thread *weatherThread = NULL;

Point p1 = {0, 0, 0, 0}, p2 = {0, 0, 0, 0};
SDL_Texture *mapTex = NULL, *startTex = NULL, *endTex = NULL;
SDL_AudioDeviceID audioDevice;
Uint8 *tickAudioBuf = NULL;
Uint32 tickAudioLen = 0;
int mapWidth, mapHeight;
// New UI Icons
SDL_Texture *iconShip = NULL, *iconApi = NULL, *iconCloud = NULL, *iconKeyboard = NULL;
SDL_Texture *iconGraphOn  = NULL, *iconGraphOff = NULL, *iconOnline   = NULL, *iconUnplug = NULL;
SDL_Texture *iconSettings = NULL, *iconLoader = NULL, *iconArrow = NULL;
int isOnline = 1; // Track API reachability

// Default Generic Vessel Parameters
char shipName[64] = "Generic Vessel";
char shipSpeed[32] = "14.0 kts";
char shipMode[32] = "N/A";
float shipSpeedKnots = 14.0f;
float fuelRate = 25.0f;
float maxWaveTol = 6.0f;
float routeDistance = 0.0f; // Total distance in km
float routeETA = 0.0f;      // Estimated time in days
float routeFuel = 0.0f;     // Fuel estimate in tons
float routeRisk = 0.0f;     // Max risk percentage (0-100)
char routeStatus[32] = "STANDBY";
int simulationMode = 0; // 0 = Real Data, 1 = Simulation Active
Uint32 lastWeatherUpdate = 0;

int showConfigWindow = 0;
float configAnimProgress = 0.0f;
int configActiveTab = 0; // 0=Vessel, 1=Env, 2=Weather, 3=Keys

// Config Inputs
int activeInputIndex = -1; // 0=Name, 1=MaxSpeed, 2=CruiseSpeed, 3=FuelRate, 4=WaveTol
char inputShipName[64] = "Generic Vessel";
char inputMaxSpeed[16] = "20";
char inputCruiseSpeed[16] = "14";
char inputFuelRate[16] = "25";
char inputWaveTol[16] = "6";

// --- Map Labels Data ---
typedef struct {
    const char *name;
    double lat;
    double lon;
    SDL_Texture *tex;
    int w, h;
} CountryLabel;

CountryLabel countries[] = {
{"Afghanistan",33.93911,67.709953},{"Albania",41.153332,20.168331},{"Algeria",28.033886,1.659626},{"American Samoa",-14.270972,-170.132217},{"Andorra",42.546245,1.601554},{"Angola",-11.202692,17.873887},{"Anguilla",18.220554,-63.068615},{"Antarctica",-75.250973,-0.071389},{"Antigua and Barbuda",17.060816,-61.796428},{"Argentina",-38.416097,-63.616672},{"Armenia",40.069099,45.038189},{"Aruba",12.52111,-69.968338},{"Australia",-25.274398,133.775136},{"Austria",47.516231,14.550072},{"Azerbaijan",40.143105,47.576927},{"Bahamas",25.03428,-77.39628},{"Bahrain",25.930414,50.637772},{"Bangladesh",23.684994,90.356331},{"Barbados",13.193887,-59.543198},{"Belarus",53.709807,27.953389},{"Belgium",50.503887,4.469936},{"Belize",17.189877,-88.49765},{"Benin",9.30769,2.315834},{"Bermuda",32.321384,-64.75737},{"Bhutan",27.514162,90.433601},{"Bolivia",-16.290154,-63.588653},{"Bosnia and Herzegovina",43.915886,17.679076},{"Botswana",-22.328474,24.684866},{"Brazil",-14.235004,-51.92528},{"Brunei",4.535277,114.727669},{"Bulgaria",42.733883,25.48583},{"Burkina Faso",12.238333,-1.561593},{"Burundi",-3.373056,29.918886},{"Cambodia",12.565679,104.990963},{"Cameroon",7.369722,12.354722},{"Canada",56.130366,-106.346771},{"Cape Verde",16.002082,-24.013197},{"Cayman Islands",19.513469,-80.566956},{"Central African Republic",6.611111,20.939444},{"Chad",15.454166,18.732207},

{"Chile",-35.675147,-71.542969},{"China",35.86166,104.195397},{"Colombia",4.570868,-74.297333},{"Comoros",-11.875001,43.872219},{"Congo (DRC)",-4.038333,21.758664},{"Congo",-0.228021,15.827659},{"Costa Rica",9.748917,-83.753428},{"Croatia",45.1,15.2},{"Cuba",21.521757,-77.781167},{"Cyprus",35.126413,33.429859},{"Czech Republic",49.817492,15.472962},{"Denmark",56.26392,9.501785},{"Djibouti",11.825138,42.590275},{"Dominica",15.414999,-61.370976},{"Dominican Republic",18.735693,-70.162651},{"Ecuador",-1.831239,-78.183406},{"Egypt",26.820553,30.802498},{"El Salvador",13.794185,-88.89653},{"Equatorial Guinea",1.650801,10.267895},{"Eritrea",15.179384,39.782334},{"Estonia",58.595272,25.013607},{"Ethiopia",9.145,40.489673},{"Finland",61.92411,25.748151},{"France",46.227638,2.213749},{"Gabon",-0.803689,11.609444},{"Gambia",13.443182,-15.310139},{"Georgia",42.315407,43.356892},{"Germany",51.165691,10.451526},{"Ghana",7.946527,-1.023194},{"Gibraltar",36.137741,-5.345374},{"Greece",39.074208,21.824312},{"Greenland",71.706936,-42.604303},{"Grenada",12.262776,-61.604171},{"Guatemala",15.783471,-90.230759},{"Guinea",9.945587,-9.696645},{"Guinea-Bissau",11.803749,-15.180413},{"Guyana",4.860416,-58.93018},{"Haiti",18.971187,-72.285215},{"Honduras",15.199999,-86.241905},{"Hungary",47.162494,19.503304},

{"Iceland",64.963051,-19.020835},{"India",20.593684,78.96288},{"Indonesia",-0.789275,113.921327},{"Iran",32.427908,53.688046},{"Iraq",33.223191,43.679291},{"Ireland",53.41291,-8.24389},{"Israel",31.046051,34.851612},{"Italy",41.87194,12.56738},{"Jamaica",18.109581,-77.297508},{"Japan",36.204824,138.252924},{"Jordan",30.585164,36.238414},{"Kazakhstan",48.019573,66.923684},{"Kenya",-0.023559,37.906193},{"Kiribati",-3.370417,-168.734039},{"Kuwait",29.31166,47.481766},{"Kyrgyzstan",41.20438,74.766098},{"Laos",19.85627,102.495496},{"Latvia",56.879635,24.603189},{"Lebanon",33.854721,35.862285},{"Lesotho",-29.609988,28.233608},{"Liberia",6.428055,-9.429499},{"Libya",26.3351,17.228331},{"Liechtenstein",47.166,9.555373},{"Lithuania",55.169438,23.881275},{"Luxembourg",49.815273,6.129583},{"Madagascar",-18.766947,46.869107},{"Malawi",-13.254308,34.301525},{"Malaysia",4.210484,101.975766},{"Maldives",3.202778,73.22068},{"Mali",17.570692,-3.996166},{"Malta",35.937496,14.375416},{"Mexico",23.634501,-102.552784},{"Moldova",47.411631,28.369885},{"Monaco",43.750298,7.412841},{"Mongolia",46.862496,103.846656},{"Montenegro",42.708678,19.37439},{"Morocco",31.791702,-7.09262},{"Mozambique",-18.665695,35.529562},{"Myanmar",21.913965,95.956223},{"Namibia",-22.95764,18.49041},

{"Nepal",28.394857,84.124008},{"Netherlands",52.132633,5.291266},{"New Zealand",-40.900557,174.885971},{"Nicaragua",12.865416,-85.207229},{"Niger",17.607789,8.081666},{"Nigeria",9.081999,8.675277},{"North Korea",40.339852,127.510093},{"Norway",60.472024,8.468946},{"Oman",21.512583,55.923255},{"Pakistan",30.375321,69.345116},{"Panama",8.537981,-80.782127},{"Papua New Guinea",-6.314993,143.95555},{"Paraguay",-23.442503,-58.443832},{"Peru",-9.189967,-75.015152},{"Philippines",12.879721,121.774017},{"Poland",51.919438,19.145136},{"Portugal",39.399872,-8.224454},{"Qatar",25.354826,51.183884},{"Romania",45.943161,24.96676},{"Russia",61.52401,105.318756},{"Rwanda",-1.940278,29.873888},{"Saudi Arabia",23.885942,45.079162},{"Senegal",14.497401,-14.452362},{"Serbia",44.016521,21.005859},{"Seychelles",-4.679574,55.491977},{"Sierra Leone",8.460555,-11.779889},{"Singapore",1.352083,103.819836},{"Slovakia",48.669026,19.699024},{"Slovenia",46.151241,14.995463},{"Somalia",5.152149,46.199616},{"South Africa",-30.559482,22.937506},{"South Korea",35.907757,127.766922},{"Spain",40.463667,-3.74922},{"Sri Lanka",7.873054,80.771797},{"Sudan",12.862807,30.217636},{"Suriname",3.919305,-56.027783},{"Sweden",60.128161,18.643501},{"Switzerland",46.818188,8.227512},{"Syria",34.802075,38.996815},{"Taiwan",23.69781,120.960515},

{"Tajikistan",38.861034,71.276093},{"Tanzania",-6.369028,34.888822},{"Thailand",15.870032,100.992541},{"Togo",8.619543,0.824782},{"Tonga",-21.178986,-175.198242},{"Trinidad and Tobago",10.691803,-61.222503},{"Tunisia",33.886917,9.537499},{"Turkey",38.963745,35.243322},{"Turkmenistan",38.969719,59.556278},{"Uganda",1.373333,32.290275},{"Ukraine",48.379433,31.16558},{"United Arab Emirates",23.424076,53.847818},{"United Kingdom",55.378051,-3.435973},{"United States",37.09024,-95.712891},{"Uruguay",-32.522779,-55.765835},{"Uzbekistan",41.377491,64.585262},{"Vanuatu",-15.376706,166.959158},{"Vatican City",41.902916,12.453389},{"Venezuela",6.42375,-66.58973},{"Vietnam",14.058324,108.277199},{"Western Sahara",24.215527,-12.885834},{"Yemen",15.552727,48.516388},{"Zambia",-13.133897,27.849332},{"Zimbabwe",-19.015438,29.154857}
};

int country_count = sizeof(countries)/sizeof(countries[0]);

typedef struct {
    const char *name;
    double lat;
    double lon;
    SDL_Texture *tex;
    int w, h;
} ContinentLabel;

ContinentLabel continents[] = {
{"North America", 54.5260, -105.2551},
{"South America", -8.7832, -55.4915},
{"Europe", 54.5260, 15.2551},
{"Africa", 1.6508, 17.6791},
{"Asia", 34.0479, 100.6197},
{"Australia", -25.2744, 133.7751},
{"Antarctica", -82.8628, 135.0000}
};

int continent_count = sizeof(continents)/sizeof(continents[0]);

// --- stb_image to SDL Helpers ---
SDL_Texture* LoadTexture(SDL_Renderer* ren, const char* path) {
    int w, h, channels;
    unsigned char* data = stbi_load(path, &w, &h, &channels, 4);
    if (!data) {
        printf("[STBI ERROR] Failed to load %s: %s\n", path, stbi_failure_reason());
        return NULL;
    }

    SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormatFrom(
        data, w, h, 32, w * 4, SDL_PIXELFORMAT_RGBA32
    );
    if (!surface) {
        printf("[SDL ERROR] CreateRGBSurface failed: %s\n", SDL_GetError());
        stbi_image_free(data);
        return NULL;
    }

    SDL_Texture* texture = SDL_CreateTextureFromSurface(ren, surface);
    SDL_FreeSurface(surface);
    stbi_image_free(data);
    return texture;
}

SDL_Surface* LoadSurface(const char* path) {
    int w, h, channels;
    unsigned char* data = stbi_load(path, &w, &h, &channels, 4);
    if (!data) {
        printf("[STBI ERROR] Failed to load %s: %s\n", path, stbi_failure_reason());
        return NULL;
    }

    SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(0, w, h, 32, SDL_PIXELFORMAT_RGBA32);
    if (!surface) {
        stbi_image_free(data);
        return NULL;
    }
    memcpy(surface->pixels, data, w * h * 4);
    stbi_image_free(data);
    return surface;
}


// --- Glowing Text Helper ---
void drawGlowingText(SDL_Renderer *ren, TTF_Font *font, const char *text, SDL_Color color, int x, int y) {
    if (!text || !*text) return;
    
    SDL_Surface *surf = TTF_RenderText_Blended(font, text, color);
    if (!surf) return;
    
    SDL_Texture *tex = SDL_CreateTextureFromSurface(ren, surf);
    if (tex) {
        SDL_Rect dst = {x, y, surf->w, surf->h};
        SDL_RenderCopy(ren, tex, NULL, &dst);
        SDL_DestroyTexture(tex);
    }
    SDL_FreeSurface(surf);
}

void drawJourneySidebar(SDL_Renderer *ren, TTF_Font *titleF, TTF_Font *labelF, TTF_Font *valueF);
void drawPlanJourneyButton(SDL_Renderer *ren, TTF_Font *font);
// --- Loading Screen Renderer ---
void drawLoadingScreen(SDL_Renderer *ren, TTF_Font *font, const char *step,
                       int progress, int total) {
  int w, h;
  SDL_GetRendererOutputSize(ren, &w, &h);

  // Dark background
  SDL_SetRenderDrawColor(ren, 5, 12, 22, 255);
  SDL_RenderClear(ren);

  // Animated scanline gradient (subtle)
  SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
  for (int y = 0; y < h; y += 4) {
    Uint8 a = (Uint8)(8 + (y % 40 == 0 ? 10 : 0));
    SDL_SetRenderDrawColor(ren, 0, 200, 255, a);
    SDL_RenderDrawLine(ren, 0, y, w, y);
  }

  if (!font) {
    SDL_RenderPresent(ren);
    return;
  }

  // AEGIS title
  SDL_Surface *title =
      TTF_RenderText_Blended(font, "AEGIS", (SDL_Color){0, 210, 255, 255});
  SDL_Texture *tTex = SDL_CreateTextureFromSurface(ren, title);
  // Scale up title
  SDL_Rect tRect = {w / 2 - title->w, h / 2 - 110, title->w * 2, title->h * 2};
  SDL_RenderCopy(ren, tTex, NULL, &tRect);
  SDL_FreeSurface(title);
  SDL_DestroyTexture(tTex);

  // Subtitle
  SDL_Surface *sub = TTF_RenderText_Blended(font, "Ocean Routing Intelligence",
                                            (SDL_Color){80, 160, 200, 200});
  SDL_Texture *sTex = SDL_CreateTextureFromSurface(ren, sub);
  SDL_Rect sRect = {w / 2 - sub->w / 2, h / 2 - 40, sub->w, sub->h};
  SDL_RenderCopy(ren, sTex, NULL, &sRect);
  SDL_FreeSurface(sub);
  SDL_DestroyTexture(sTex);

  // Progress bar background
  SDL_Rect barBg = {w / 2 - 220, h / 2 + 20, 440, 8};
  SDL_SetRenderDrawColor(ren, 20, 45, 65, 255);
  SDL_RenderFillRect(ren, &barBg);

  // Progress bar fill with glow
  int fillW = (progress * 440) / (total > 0 ? total : 1);
  SDL_Rect barFill = {barBg.x, barBg.y, fillW, 8};
  SDL_SetRenderDrawColor(ren, 0, 200, 255, 255);
  SDL_RenderFillRect(ren, &barFill);
  // glow line above
  SDL_SetRenderDrawColor(ren, 150, 230, 255, 80);
  SDL_RenderDrawLine(ren, barFill.x, barFill.y - 1, barFill.x + fillW,
                     barFill.y - 1);

  // Step label
  SDL_Surface *stepSurf =
      TTF_RenderText_Blended(font, step, (SDL_Color){180, 220, 255, 255});
  SDL_Texture *stepTex = SDL_CreateTextureFromSurface(ren, stepSurf);
  SDL_Rect stepRect = {w / 2 - stepSurf->w / 2, h / 2 + 40, stepSurf->w,
                       stepSurf->h};
  SDL_RenderCopy(ren, stepTex, NULL, &stepRect);
  SDL_FreeSurface(stepSurf);
  SDL_DestroyTexture(stepTex);

  SDL_RenderPresent(ren);
}


unsigned char *collisionGrid = NULL;
WeatherCell *weatherGrid = NULL;
int gridW, gridH;
GridPos *finalPath = NULL;
int pathLen = 0;

// --- Coordinate Helpers (Mapped to User Bounding Box) ---
int worldToScreenX(float wx) {
  return (int)((wx - camX) * zoom + winWidth / 2);
}
int worldToScreenY(float wy) {
  return (int)((wy - camY) * zoom + winHeight / 2 + TOPBAR);
}
float screenToWorldX(int sx) { return (sx - winWidth / 2) / zoom + camX; }
float screenToWorldY(int sy) {
  return (sy - TOPBAR - winHeight / 2) / zoom + camY;
}
float worldToPixelX(float wx) { return wx + mapWidth / 2.0f; }
float worldToPixelY(float wy) { return wy + mapHeight / 2.0f; }

float pixelToLat(float pixel_y) {
  // Derived from data points: m = -1296.0, c = 3988.0
  return (2.0f * atanf(expf((pixel_y - 3988.0f) / -1296.0f)) -
          (float)M_PI / 2.0f) *
         180.0f / (float)M_PI;
}

float pixelToLon(float pixel_x) {
  // Derived from data points: scale = 22.7835, c = 3870.0
  return (pixel_x - 3870.0f) / 22.7835f;
}

float lonToPixelX(float lon) { return (lon * 22.7835f) + 3870.0f; }

float latToPixelY(float lat) {
  // Inverse of pixelToLat: lat = (2 * atan(exp((y - 3988) / -1296)) - PI/2) *
  // 180 / PI Convert lat to radians
  float latRad = lat * (float)M_PI / 180.0f;
  // latRad + PI/2 = 2 * atan(...)
  // (latRad + PI/2) / 2 = atan(...)
  // tan( (latRad + PI/2) / 2 ) = exp((y - 3988) / -1296)
  // ln( tan(...) ) = (y - 3988) / -1296
  // -1296 * ln( tan(...) ) = y - 3988
  // y = 3988 - 1296 * ln( tan( (latRad + PI/2) / 2 ) )
  return 3988.0f - 1296.0f * logf(tanf((latRad + (float)M_PI / 2.0f) / 2.0f));
}

void formatCoord(char *buf, size_t size, float lat, float lon) {
  snprintf(buf, size, "%.4f%c %.4f%c", fabs(lat), lat >= 0 ? 'N' : 'S',
           fabs(lon), lon >= 0 ? 'E' : 'W');
}



void wrapCamera() {
  float half = mapWidth * 0.5f;
  if (camX > half)
    camX -= mapWidth;
  else if (camX < -half)
    camX += mapWidth;
}

void drawRoundedRect(SDL_Renderer *ren, SDL_Rect rect, int radius,
                     SDL_Color color, int glossy) {
  // Set blend mode based on alpha
  if (color.a < 255) {
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
  }

  SDL_SetRenderDrawColor(ren, color.r, color.g, color.b, color.a);
  for (int y = 0; y < rect.h; y++) {
    int startX = 0;
    int endX = rect.w;

    if (y < radius) {
      int dy = radius - y;
      int dx = (int)(radius - sqrtf((float)(radius * radius) - (float)(dy * dy)));
      startX = dx;
      endX = rect.w - dx;
    } else if (y >= rect.h - radius) {
      int dy = y - (rect.h - radius) + 1;
      int dx = (int)(radius - sqrtf((float)(radius * radius) - (float)(dy * dy)));
      startX = dx;
      endX = rect.w - dx;
    }
    SDL_RenderDrawLine(ren, rect.x + startX, rect.y + y, rect.x + endX - 1, rect.y + y);
  }

  // Add glossy effect
  if (glossy) {
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    int glossHeight = rect.h / 3;
    for (int y = 0; y < glossHeight; y++) {
      int alpha = (int)(25.0f * (1.0f - (float)y / glossHeight));
      SDL_SetRenderDrawColor(ren, 255, 255, 255, alpha);

      int startX = 0;
      int endX = rect.w;
      if (y < radius) {
        int dy = radius - y;
        int dx = (int)(radius - sqrtf((float)(radius * radius) - (float)(dy * dy)));
        startX = dx;
        endX = rect.w - dx;
      }
      SDL_RenderDrawLine(ren, rect.x + startX, rect.y + y, rect.x + endX - 1, rect.y + y);
    }
  }

  // Restore blend mode
  if (color.a < 255 || glossy) {
      SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_NONE);
  }
}

void drawRoundedRectBorder(SDL_Renderer *ren, SDL_Rect rect, int radius,
                           SDL_Color color) {
  SDL_SetRenderDrawColor(ren, color.r, color.g, color.b, color.a);

  // Draw straight edges
  SDL_RenderDrawLine(ren, rect.x + radius, rect.y, rect.x + rect.w - radius - 1,
                     rect.y); // Top
  SDL_RenderDrawLine(ren, rect.x + radius, rect.y + rect.h - 1,
                     rect.x + rect.w - radius - 1,
                     rect.y + rect.h - 1); // Bottom
  SDL_RenderDrawLine(ren, rect.x, rect.y + radius, rect.x,
                     rect.y + rect.h - radius - 1); // Left
  SDL_RenderDrawLine(ren, rect.x + rect.w - 1, rect.y + radius,
                     rect.x + rect.w - 1,
                     rect.y + rect.h - radius - 1); // Right

  // Draw rounded corners - only the arc portions
  for (int w = 0; w < radius * 2; w++) {
    for (int h = 0; h < radius * 2; h++) {
      int dx = radius - w;
      int dy = radius - h;
      int dist = dx * dx + dy * dy;
      // Draw only the border pixels (at radius distance)
      if (dist <= radius * radius && dist > (radius - 1) * (radius - 1)) {
        // Top-left corner (only draw if in top-left quadrant)
        if (w <= radius && h <= radius) {
          SDL_RenderDrawPoint(ren, rect.x + w, rect.y + h);
        }
        // Top-right corner (only draw if in top-right quadrant)
        if (w >= radius && h <= radius) {
          SDL_RenderDrawPoint(ren, rect.x + rect.w - (radius * 2 - w),
                              rect.y + h);
        }
        // Bottom-left corner (only draw if in bottom-left quadrant)
        if (w <= radius && h >= radius) {
          SDL_RenderDrawPoint(ren, rect.x + w,
                              rect.y + rect.h - (radius * 2 - h));
        }
        // Bottom-right corner (only draw if in bottom-right quadrant)
        if (w >= radius && h >= radius) {
          SDL_RenderDrawPoint(ren, rect.x + rect.w - (radius * 2 - w),
                              rect.y + rect.h - (radius * 2 - h));
        }
      }
    }
  }
}

float distPointToSegment(float px, float py, float x1, float y1, float x2, float y2) {
  float l2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
  if (l2 == 0.0)
    return sqrtf((px - x1) * (px - x1) + (py - y1) * (py - y1));
  float t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / l2;
  t = fmaxf(0, fminf(1, t));
  return sqrtf((px - (x1 + t * (x2 - x1))) * (px - (x1 + t * (x2 - x1))) +
               (py - (y1 + t * (y2 - y1))) * (py - (y1 + t * (y2 - y1))));
}

void drawPlanJourneyButton(SDL_Renderer *ren, TTF_Font *font) {
    // Assuming showJourneyPanel, journeyPanelAnim, drawGlowingText are defined elsewhere
    // For this change, we'll assume they are available.
    // If not, this code would need further context.
    if (showJourneyPanel || journeyPanelAnim > 0.01f) return;
    
    SDL_Rect btn = {20, winHeight - 110, 200, 45};
    
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(ren, 10, 25, 45, 200);
    drawRoundedRect(ren, btn, 8, (SDL_Color){10, 25, 45, 200}, 1);
    drawRoundedRectBorder(ren, btn, 8, (SDL_Color){0, 180, 255, 255});
    
    drawGlowingText(ren, font, "PLAN JOURNEY", (SDL_Color){255, 255, 255, 255}, btn.x + 35, btn.y + 12);
}

void drawJourneySidebar(SDL_Renderer *ren, TTF_Font *titleF, TTF_Font *labelF, TTF_Font *valueF) {
    // Assuming journeyPanelAnim, showJourneyPanel, activeInputIndex, startCoordTxt, endCoordTxt, depDateTxt,
    // pathLen, routeETA, routeRisk, pathHoverIndex, finalPath, pathHoverT, mapWidth, mapHeight,
    // worldToScreenX, worldToScreenY, pixelToLat, pixelToLon, worldToPixelY, worldToPixelX, drawGlowingText
    // are defined elsewhere.
    // For this change, we'll assume they are available.
    // If not, this code would need further context.

    if (journeyPanelAnim <= 0.01f) return;
    
    int panelW = 320;
    int panelX = (int)(-panelW + (panelW * journeyPanelAnim));
    SDL_Rect sidebar = {panelX, 0, panelW, winHeight};
    
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(ren, 10, 20, 30, 240);
    SDL_RenderFillRect(ren, &sidebar);
    SDL_SetRenderDrawColor(ren, 0, 180, 255, 120);
    SDL_RenderDrawLine(ren, panelX + panelW - 1, 0, panelX + panelW - 1, winHeight);
    
    int curY = 30;
    drawGlowingText(ren, titleF, "JOURNEY PLANNER", (SDL_Color){255,255,255,255}, panelX + 30, curY);
    
    // Close 'X'
    SDL_Rect closeR = {panelX + panelW - 40, 25, 20, 20};
    SDL_SetRenderDrawColor(ren, 255, 80, 80, 255);
    SDL_RenderDrawLine(ren, closeR.x, closeR.y, closeR.x + closeR.w, closeR.y + closeR.h);
    SDL_RenderDrawLine(ren, closeR.x + closeR.w, closeR.y, closeR.x, closeR.y + closeR.h);
    
    curY += 70;
    // Start Field
    drawGlowingText(ren, labelF, "START POINT", (SDL_Color){0, 180, 255, 255}, panelX + 30, curY);
    curY += 25;
    SDL_Rect sBox = {panelX + 30, curY, panelW - 60, 35};
    SDL_SetRenderDrawColor(ren, 20, 30, 40, 255);
    SDL_RenderFillRect(ren, &sBox);
    SDL_Color sBoxBorder = activeInputIndex == 5 ? (SDL_Color){0, 255, 255, 255} : (SDL_Color){0, 180, 255, 150};
    SDL_SetRenderDrawColor(ren, sBoxBorder.r, sBoxBorder.g, sBoxBorder.b, sBoxBorder.a);
    SDL_RenderDrawRect(ren, &sBox);
    drawGlowingText(ren, labelF, startCoordTxt, (SDL_Color){230, 230, 230, 255}, sBox.x + 10, sBox.y + 8);
    
    curY += 60;
    // End Field
    drawGlowingText(ren, labelF, "END POINT", (SDL_Color){0, 180, 255, 255}, panelX + 30, curY);
    curY += 25;
    SDL_Rect eBox = {panelX + 30, curY, panelW - 60, 35};
    SDL_SetRenderDrawColor(ren, 20, 30, 40, 255);
    SDL_RenderFillRect(ren, &eBox);
    SDL_Color eBoxBorder = activeInputIndex == 6 ? (SDL_Color){0, 255, 255, 255} : (SDL_Color){0, 180, 255, 150};
    SDL_SetRenderDrawColor(ren, eBoxBorder.r, eBoxBorder.g, eBoxBorder.b, eBoxBorder.a);
    SDL_RenderDrawRect(ren, &eBox);
    drawGlowingText(ren, labelF, endCoordTxt, (SDL_Color){230, 230, 230, 255}, eBox.x + 10, eBox.y + 8);

    curY += 60;
    // Date Field
    drawGlowingText(ren, labelF, "DEPARTURE DATE", (SDL_Color){0, 180, 255, 255}, panelX + 30, curY);
    curY += 25;
    SDL_Rect dBox = {panelX + 30, curY, panelW - 60, 35};
    SDL_SetRenderDrawColor(ren, 20, 30, 40, 255);
    SDL_RenderFillRect(ren, &dBox);
    SDL_Color dBoxBorder = activeInputIndex == 7 ? (SDL_Color){0, 255, 255, 255} : (SDL_Color){0, 180, 255, 150};
    SDL_SetRenderDrawColor(ren, dBoxBorder.r, dBoxBorder.g, dBoxBorder.b, dBoxBorder.a);
    SDL_RenderDrawRect(ren, &dBox);
    drawGlowingText(ren, labelF, depDateTxt, (SDL_Color){230, 230, 230, 255}, dBox.x + 10, dBox.y + 8);

    curY += 70;
    // Start Journey Button
    SDL_Rect sjBtn = {panelX + 30, curY, panelW - 60, 45};
    SDL_SetRenderDrawColor(ren, 40, 180, 80, 255);
    SDL_RenderFillRect(ren, &sjBtn);
    SDL_SetRenderDrawColor(ren, 100, 255, 100, 255);
    SDL_RenderDrawRect(ren, &sjBtn);
    drawGlowingText(ren, labelF, "START JOURNEY", (SDL_Color){255,255,255,255}, sjBtn.x + (panelW-60)/2 - 60, sjBtn.y + 12);

    curY += 65;
    // Reset Button
    SDL_Rect rBtn = {panelX + 30, curY, panelW - 60, 40};
    SDL_SetRenderDrawColor(ren, 200, 50, 50, 60);
    SDL_RenderFillRect(ren, &rBtn);
    SDL_SetRenderDrawColor(ren, 255, 80, 80, 200);
    SDL_RenderDrawRect(ren, &rBtn);
    drawGlowingText(ren, labelF, "RESET", (SDL_Color){255,255,255,255}, rBtn.x + (panelW-60)/2 - 30, rBtn.y + 10);

    curY += 80;
    // Analysis
    if (pathLen > 0) {
        drawGlowingText(ren, valueF, "ANALYSIS", (SDL_Color){0, 180, 255, 255}, panelX + 30, curY);
        curY += 40;
        
        // --- Calculate Arrival Date ---
        struct tm tm_dep = {0};
        int year, month, day, hour, min;
        if (sscanf(depDateTxt, "%d-%d-%d %d:%d", &year, &month, &day, &hour, &min) >= 3) {
            tm_dep.tm_year = year - 1900;
            tm_dep.tm_mon = month - 1;
            tm_dep.tm_mday = day;
            tm_dep.tm_hour = (sscanf(depDateTxt, "%*d-%*d-%*d %d:%d", &hour, &min) == 2) ? hour : 12;
            tm_dep.tm_min = (sscanf(depDateTxt, "%*d-%*d-%*d %*d:%d", &min) == 1) ? min : 0;
            tm_dep.tm_isdst = -1;
            
            time_t dep_time = mktime(&tm_dep);
            if (dep_time != -1) {
                time_t arr_time = dep_time + (time_t)(routeETA * 86400); 
                struct tm *tm_arr = localtime(&arr_time);
                char arrStr[64];
                strftime(arrStr, sizeof(arrStr), "Arrival: %b %d, %H:%M", tm_arr);
                drawGlowingText(ren, labelF, arrStr, (SDL_Color){200, 255, 200, 255}, panelX + 30, curY);
            }
        } else {
            char arrStr[32];
            snprintf(arrStr, sizeof(arrStr), "Arrival: +%.1f days", routeETA);
            drawGlowingText(ren, labelF, arrStr, (SDL_Color){200, 200, 200, 255}, panelX + 30, curY);
        }
        
        curY += 30;
        char riskStr[64];
        snprintf(riskStr, sizeof(riskStr), "Max Risk: %.1f%%", routeRisk);
        SDL_Color rCol = routeRisk > 50.0f ? (SDL_Color){255, 100, 100, 255} : (SDL_Color){200, 200, 200, 255};
        drawGlowingText(ren, labelF, riskStr, rCol, panelX + 30, curY);

        // --- Pointer Details Section ---
        if (pathHoverIndex != -1) {
            curY += 60;
            drawGlowingText(ren, valueF, "POINT DETAILS", (SDL_Color){0, 180, 255, 255}, panelX + 30, curY);
            curY += 35;
            
            // Calculate Point Time
            char timeStr[64] = "Time: --:--";
            float distAtPoint = pathDistances[pathHoverIndex];
            if (pathHoverIndex < pathLen - 1) {
                float segDist = pathDistances[pathHoverIndex+1] - pathDistances[pathHoverIndex];
                distAtPoint += segDist * pathHoverT;
            }
            float etaAtPoint = distAtPoint / (shipSpeedKnots * 1.852f * 24.0f); // days
            
            struct tm tm_p = {0};
            if (sscanf(depDateTxt, "%d-%d-%d %d:%d", &year, &month, &day, &hour, &min) >= 3) {
                tm_p.tm_year = year - 1900; tm_p.tm_mon = month - 1; tm_p.tm_mday = day;
                tm_p.tm_hour = hour; tm_p.tm_min = min; tm_p.tm_isdst = -1;
                time_t p_dep = mktime(&tm_p);
                if (p_dep != -1) {
                    time_t p_time = p_dep + (time_t)(etaAtPoint * 86400);
                    struct tm *tm_actual = localtime(&p_time);
                    strftime(timeStr, sizeof(timeStr), "Time: %b %d, %H:%M", tm_actual);
                }
            }
            
            char d1[64], d2[64], d3[64];
            snprintf(d1, sizeof(d1), "Wind: %.1f km/h", hoverWind);
            snprintf(d2, sizeof(d2), "Waves: %.1f m", (hoverWave < 0 ? 0 : hoverWave));
            snprintf(d3, sizeof(d3), "Dir: %.0f deg", hoverDir);
            
            drawGlowingText(ren, labelF, timeStr, (SDL_Color){255, 255, 255, 255}, panelX + 30, curY); curY += 25;
            drawGlowingText(ren, labelF, d1, (SDL_Color){200, 220, 255, 255}, panelX + 30, curY); curY += 25;
            drawGlowingText(ren, labelF, d2, (SDL_Color){200, 220, 255, 255}, panelX + 30, curY); curY += 25;
            drawGlowingText(ren, labelF, d3, (SDL_Color){200, 220, 255, 255}, panelX + 30, curY);
        }
    }

    // Path Hover Tooltip
    if (pathHoverIndex != -1 && finalPath) {
        float x1 = finalPath[pathHoverIndex].c * GRID_SCALE - mapWidth / 2.0f;
        float y1 = finalPath[pathHoverIndex].r * GRID_SCALE - mapHeight / 2.0f;
        float x2 = finalPath[pathHoverIndex+1].c * GRID_SCALE - mapWidth / 2.0f;
        float y2 = finalPath[pathHoverIndex+1].r * GRID_SCALE - mapHeight / 2.0f;
        float hx = x1 + (x2 - x1) * pathHoverT;
        float hy = y1 + (y2 - y1) * pathHoverT;
        
        int tipX = worldToScreenX(hx) + 20;
        int tipY = worldToScreenY(hy) - 40;
        char tip[64];
        snprintf(tip, sizeof(tip), "%.3f, %.3f", pixelToLat(worldToPixelY(hy)), pixelToLon(worldToPixelX(hx)));
        
        // Push tooltip away from sidebar if it would overlap
        if (tipX < panelX + panelW + 20) {
            tipX = panelX + panelW + 20;
        }

        SDL_Rect tipRect = {tipX, tipY, 160, 30};
        SDL_SetRenderDrawColor(ren, 10, 20, 30, 230);
        SDL_RenderFillRect(ren, &tipRect);
        SDL_SetRenderDrawColor(ren, 0, 200, 255, 180);
        SDL_RenderDrawRect(ren, &tipRect);
        drawGlowingText(ren, labelF, tip, (SDL_Color){220, 220, 220, 255}, tipX + 10, tipY + 6);
    }
}

void drawToggle(SDL_Renderer *ren, TTF_Font *labelF, TTF_Font *smallF, const char *label, const char *sub, int x, int y, int w, int active) {
    // Label
    SDL_Surface *s = TTF_RenderText_Blended(labelF, label, (SDL_Color){255, 255, 255, 255});
    SDL_Texture *t = SDL_CreateTextureFromSurface(ren, s);
    SDL_Rect r = {x, y, s->w, s->h};
    SDL_RenderCopy(ren, t, NULL, &r);
    SDL_FreeSurface(s);
    SDL_DestroyTexture(t);
    
    // Subtitle
    if (sub) {
        SDL_Surface *ss = TTF_RenderText_Blended(smallF, sub, (SDL_Color){0, 180, 255, 200});
        SDL_Texture *st = SDL_CreateTextureFromSurface(ren, ss);
        SDL_Rect sr = {x, y + 22, ss->w, ss->h};
        SDL_RenderCopy(ren, st, NULL, &sr);
        SDL_FreeSurface(ss);
        SDL_DestroyTexture(st);
    }
    
    // Switch Background
    SDL_Rect sw = {x + w - 50, y + 5, 44, 22};
    drawRoundedRect(ren, sw, 11, active ? (SDL_Color){0, 255, 255, 255} : (SDL_Color){40, 50, 60, 255}, 0);
    
    // Switch Knob
    SDL_Rect knob = {active ? (sw.x + sw.w - 20) : (sw.x + 2), sw.y + 2, 18, 18};
    drawRoundedRect(ren, knob, 9, (SDL_Color){255, 255, 255, 255}, 0);
}

static void drawBox(SDL_Renderer *ren, TTF_Font *labelFont, TTF_Font *font, TTF_Font *smallFont, const char *label, const char *val, const char* unit, int x, int y, int w, int active) {
   SDL_Surface *ls = TTF_RenderText_Blended(labelFont, label, (SDL_Color){150, 150, 150, 255});
   SDL_Texture *lt = SDL_CreateTextureFromSurface(ren, ls);
   SDL_Rect lr = {x, y, ls->w, ls->h};
   SDL_RenderCopy(ren, lt, NULL, &lr);
   SDL_FreeSurface(ls);
   SDL_DestroyTexture(lt);

   SDL_Rect boxR = {x, y+25, w, 35};
   drawRoundedRect(ren, boxR, 4, (SDL_Color){25, 28, 35, 255}, 0);
   
   if (active) {
       drawRoundedRectBorder(ren, boxR, 4, (SDL_Color){0, 255, 255, 255}); // Cyan highlight
   } else {
       drawRoundedRectBorder(ren, boxR, 4, (SDL_Color){50, 50, 50, 255});
   }

   if (val[0] != '\0') {
       SDL_Surface *vs = TTF_RenderText_Blended(font, val, (SDL_Color){255, 255, 255, 255});
       SDL_Texture *vt = SDL_CreateTextureFromSurface(ren, vs);
       SDL_Rect vr = {x + 10, y + 33, vs->w, vs->h};
       SDL_RenderCopy(ren, vt, NULL, &vr);
       SDL_FreeSurface(vs);
       SDL_DestroyTexture(vt);
   }

   // Cursor marker if active
   if (active) {
       int textW = 0;
       if (val[0] != '\0') TTF_SizeText(font, val, &textW, NULL);
       if ((SDL_GetTicks() % 1000) < 500) {
           SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
           SDL_RenderDrawLine(ren, x + 10 + textW + 2, y + 30, x + 10 + textW + 2, y + 55);
       }
   }

   if (unit) {
     SDL_Surface *us = TTF_RenderText_Blended(smallFont, unit, (SDL_Color){100, 100, 100, 255});
     SDL_Texture *ut = SDL_CreateTextureFromSurface(ren, us);
     SDL_Rect ur = {x + w - us->w - 10, y + 35, us->w, us->h};
     SDL_RenderCopy(ren, ut, NULL, &ur);
     SDL_FreeSurface(us);
     SDL_DestroyTexture(ut);
   }
}

static void drawBlock(SDL_Renderer *ren, TTF_Font *smallFont, const char *title, int x, int y, int w, int h) {
   SDL_Rect blkR = {x, y, w, h};
   drawRoundedRect(ren, blkR, 6, (SDL_Color){25, 28, 35, 255}, 0);
   drawRoundedRectBorder(ren, blkR, 6, (SDL_Color){40, 45, 55, 255});
   
   SDL_Surface *ts = TTF_RenderText_Blended(smallFont, title, (SDL_Color){0, 180, 255, 255});
   SDL_Texture *tt = SDL_CreateTextureFromSurface(ren, ts);
   SDL_Rect tr = {x + 15, y + 15, ts->w, ts->h};
   SDL_RenderCopy(ren, tt, NULL, &tr);
   SDL_FreeSurface(ts);
   SDL_DestroyTexture(tt);
}

static void drawKeyRow(SDL_Renderer *ren, TTF_Font *labelFont, TTF_Font *smallFont, const char *label, const char *keyName, const char *extra, const char *desc, int x, int y, int blkW) {
   SDL_Surface *ls = TTF_RenderText_Blended(labelFont, label, (SDL_Color){200, 200, 200, 255});
   SDL_Texture *lt = SDL_CreateTextureFromSurface(ren, ls);
   SDL_Rect lr = {x + 15, y, ls->w, ls->h};
   SDL_RenderCopy(ren, lt, NULL, &lr);
   SDL_FreeSurface(ls);
   SDL_DestroyTexture(lt);

   if (desc) {
     SDL_Surface *ds = TTF_RenderText_Blended(smallFont, desc, (SDL_Color){100, 100, 100, 255});
     SDL_Texture *dt = SDL_CreateTextureFromSurface(ren, ds);
     SDL_Rect dr = {x + 15, y + 20, ds->w, ds->h};
     SDL_RenderCopy(ren, dt, NULL, &dr);
     SDL_FreeSurface(ds);
     SDL_DestroyTexture(dt);
   }

   SDL_Surface *ks = TTF_RenderText_Blended(smallFont, keyName, (SDL_Color){255, 255, 255, 255});
   SDL_Texture *kt = SDL_CreateTextureFromSurface(ren, ks);
   int kx = x + blkW - ks->w - 20 - (extra ? 45 : 0);
   
   SDL_Rect keyBg = {kx - 5, y - 2, ks->w + 10, 20};
   drawRoundedRectBorder(ren, keyBg, 3, (SDL_Color){100, 100, 100, 255});
   SDL_Rect kr = {kx, y, ks->w, ks->h};
   SDL_RenderCopy(ren, kt, NULL, &kr);
   SDL_FreeSurface(ks);
   SDL_DestroyTexture(kt);

   if (extra) {
     SDL_Surface *es = TTF_RenderText_Blended(smallFont, extra, (SDL_Color){200, 200, 200, 255});
     SDL_Texture *et = SDL_CreateTextureFromSurface(ren, es);
     SDL_Rect er = {kx + ks->w + 10, y, es->w, es->h};
     SDL_RenderCopy(ren, et, NULL, &er);
     SDL_FreeSurface(es);
     SDL_DestroyTexture(et);
   }
}

// --- Weather Implementation ---

// --- Open-Meteo API Integration ---
// Helper: simple parser (replaced redundant one)
void applyRealWeather(float lat, float lon, float windSpeed, float dir,
                      float wave) {
  if (!weatherGrid)
    return;
  printf("Applying Marine: %.1f km/h, %.1fm, %.0f deg at %.2f, %.2f\n",
         windSpeed, wave, dir, lat, lon);

  for (int r = 0; r < gridH; r++) {
    for (int c = 0; c < gridW; c++) {
      float plat = pixelToLat(r * GRID_SCALE);
      float plon = pixelToLon(c * GRID_SCALE);

      // Distance in degrees (approx)
      float dist = sqrtf(pow(plat - lat, 2) + pow(plon - lon, 2));

      // Apply storm radius (e.g., 2.5 degrees)
      if (dist < 2.5f) {
        if (windSpeed > weatherGrid[r * gridW + c].windSpeed) {
          weatherGrid[r * gridW + c].windSpeed = windSpeed;
          weatherGrid[r * gridW + c].windDir = dir;
          weatherGrid[r * gridW + c].waveHeight = wave;
        }
      }
    }
  }
}

float parseJSONValue(const char *json, const char *key) {
  // Try to find the start of the actual data block to skip units/metadata
  const char *dataStart = strstr(json, "\"current\":");
  if (!dataStart) dataStart = json; // Fallback to whole string if block is missing

  char *pos = strstr(dataStart, key);
  if (pos) {
    char *val = pos + strlen(key);
    while (*val == ' ' || *val == ':' || *val == '"')
      val++;
    if (strncmp(val, "null", 4) == 0 || strncmp(val, "undefined", 9) == 0)
      return -1.0f; // Handle invalid values explicitly
    return (float)atof(val);
  }
  return -1.0f;
}

void fetchWeatherAndApply(float lat, float lon) {
  char cmd[1024];
  char outFile[128];
  // Overwrite a single file to keep project directory clean
  snprintf(outFile, sizeof(outFile), "latest_weather.json");

  snprintf(cmd, sizeof(cmd),
           "curl -s "
           "\"https://api.open-meteo.com/v1/"
           "forecast?latitude=%.2f&longitude=%.2f&current=wind_speed_10m,"
           "wind_direction_10m&models=best_match\" > %s",
           lat, lon, outFile);

  printf("\n[API CALL] Marine fetch triggered\n");
  printf("[API CALL]   Endpoint : https://marine-api.open-meteo.com/v1/marine\n");
  printf("[API CALL]   Latitude : %.4f\n", lat);
  printf("[API CALL]   Longitude: %.4f\n", lon);
  printf("[API CALL]   Params   : wind_wave_height, wind_speed_10m, wind_direction_10m\n");
  printf("[API CALL]   Saving response --> %s\n", outFile);

  int ret = system(cmd);
  if (ret != 0) {
    printf("[API ERROR] curl command failed (exit code %d).\n", ret);
    isOnline = 0;
    return;
  }
  isOnline = 1;

  FILE *f = fopen(outFile, "r");
  if (!f) {
    printf("[API ERROR] Could not open response file: %s\n", outFile);
    return;
  }

  char buffer[4096];
  size_t len = fread(buffer, 1, sizeof(buffer) - 1, f);
  buffer[len] = '\0';
  fclose(f);

  printf("[API DATA]  Response (%zu bytes): %s\n", len, buffer);

  float wind = parseJSONValue(buffer, "wind_speed_10m");
  float dir  = parseJSONValue(buffer, "wind_direction_10m");
  float wave = parseJSONValue(buffer, "wind_wave_height");

  if (wind >= 0) {
    simulationMode = 0;
    printf("[API DATA]  Parsed -> Wind: %.1f km/h | Dir: %.0f deg | Wave: %.1f m\n",
           wind, dir, wave);
    applyRealWeather(lat, lon, wind, dir, wave);
             // snprintf(infoText, ...) removed
  } else {
    simulationMode = 1;
    float simWind = 40.0f + (rand() % 50);
    float simWave = 1.5f + (simWind / 20.0f);
    float simDir  = (float)(rand() % 360);
    printf("[API DATA]  No data for location. Falling back to simulated storm: %.1f km/h\n",
           simWind);
    applyRealWeather(lat, lon, simWind, simDir, simWave);
  }
}

void scanOceanWeather() {
  float lats[] = {45.0f, -25.0f, -20.0f, 35.0f, -30.0f, 35.0f};
  float lons[] = {-30.0f, -15.0f, 75.0f, 160.0f, -140.0f, 15.0f};
  int count = 6;
  const char *dataFile = "marine_scan_data.json";

  char url[4096];
  snprintf(url, sizeof(url),
           "https://api.open-meteo.com/v1/forecast?latitude=");
  for (int i = 0; i < count; i++) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.2f%s", lats[i], (i < count - 1) ? "," : "");
    strcat(url, buf);
  }
  strcat(url, "&longitude=");
  for (int i = 0; i < count; i++) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.2f%s", lons[i], (i < count - 1) ? "," : "");
    strcat(url, buf);
  }
  strcat(url, "&current=wind_speed_10m,wind_direction_10m&models=best_match");

  char cmd[8192];
  snprintf(cmd, sizeof(cmd), "curl -s \"%s\" > %s", url, dataFile);

  printf("\n[API CALL] Global marine weather scan triggered\n");
  printf("[API CALL]   Endpoint : %s\n", url);
  printf("[API CALL]   Locations: %d ocean points\n", count);
  printf("[API CALL]   Saving response --> %s\n", dataFile);
  // No longer needed: snprintf(infoText, sizeof(infoText), "Scanning Marine Data...");

  int ret = system(cmd);
  if (ret != 0) {
    printf("[API ERROR] curl command failed (exit code %d).\n", ret);
    isOnline = 0;
    return;
  }
  isOnline = 1;

  // Read from the saved data file
  FILE *f = fopen(dataFile, "r");
  if (!f) {
    printf("[API ERROR] Could not open response file: %s\n", dataFile);
    return;
  }

  char buffer[32768];
  size_t bytesRead = fread(buffer, 1, sizeof(buffer) - 1, f);
  buffer[bytesRead] = '\0';
  fclose(f);

  printf("[API DATA]  Response (%zu bytes) stored in %s\n", bytesRead, dataFile);

  // Multi-point parsing from saved file
  const char *cursor = strstr(buffer, "\"current\":");
  if (!cursor) cursor = buffer;

  int applied = 0;
  for (int i = 0; i < count; i++) {
    float wind = parseJSONValue(cursor, "\"wind_speed_10m\"");
    if (wind >= 0) {
      char *next = strstr(cursor, "\"wind_speed_10m\"");
      if (next) {
        float dir  = parseJSONValue(next, "\"wind_direction_10m\"");
        float wave = wind / 25.0f; // Approximate wave height from wind if marine API fails
        printf("[API DATA]  Point %d (%.2f, %.2f): Wind %.1f km/h | Dir %.0f deg | Wave %.1f m\n",
               i + 1, lats[i], lons[i], wind, dir, wave);
        applyRealWeather(lats[i], lons[i], wind, dir, wave);
        cursor = next + 20;
        applied++;
      } else {
        // Attempt to skip to next object in array if key not found in current block
        cursor += 20;
      }
    } else {
        // Skip current and look for next
        char *nextObj = strstr(cursor + 1, "{");
        if (nextObj) cursor = nextObj;
        else cursor += 20;
    }
  }
  printf("[API DATA]  Applied weather for %d/%d ocean points.\n", applied, count);
  // No longer needed: snprintf(infoText, sizeof(infoText), "Global Marine Scan Complete.");
}

int samplePathWeatherThread(void *data) {
  if (!finalPath || pathLen < 5) {
      weatherThreadActive = 0;
      return 0;
  }
  
  int numSamples = 8;
  if (pathLen < 8) numSamples = pathLen;
  float lats[8], lons[8];
  
  for (int i = 0; i < numSamples; i++) {
    int idx = (i * (pathLen - 1)) / (numSamples - 1);
    float wx = finalPath[idx].c * GRID_SCALE - mapWidth / 2.0f;
    float wy = finalPath[idx].r * GRID_SCALE - mapHeight / 2.0f;
    lats[i] = pixelToLat(worldToPixelY(wy));
    lons[i] = pixelToLon(worldToPixelX(wx));
  }
  
  char url[4096];
  snprintf(url, sizeof(url), "https://api.open-meteo.com/v1/forecast?latitude=");
  for (int i = 0; i < numSamples; i++) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.2f%s", lats[i], (i < numSamples - 1) ? "," : "");
    strcat(url, buf);
  }
  strcat(url, "&longitude=");
  for (int i = 0; i < numSamples; i++) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.2f%s", lons[i], (i < numSamples - 1) ? "," : "");
    strcat(url, buf);
  }
  strcat(url, "&current=wind_speed_10m,wind_direction_10m&models=best_match");
  
  char cmd[8192];
  const char *dataFile = "latest_weather.json";
  snprintf(cmd, sizeof(cmd), "curl -s \"%s\" > %s", url, dataFile);
  
  printf("\n[SMART ROUTE] Path Weather Verification triggered (ASYNCHRONOUS)\n");
  printf("[SMART ROUTE]   Endpoint : %s\n", url);
  printf("[SMART ROUTE]   Locations: %d path waypoints\n", numSamples);
  
  int ret = system(cmd);
  if (ret != 0) {
    printf("[SMART ERROR] curl failed.\n");
    isOnline = 0; // Mark as offline
    weatherThreadActive = 0;
    return 0;
  }
  isOnline = 1; // Mark as online
  
  FILE *f = fopen(dataFile, "r");
  if (!f) {
      weatherThreadActive = 0;
      return 0;
  }
  char buffer[32768];
  size_t bytesRead = fread(buffer, 1, sizeof(buffer) - 1, f);
  buffer[bytesRead] = '\0';
  fclose(f);
  
  const char *cursor = strstr(buffer, "\"current\":");
  if (!cursor) cursor = buffer;
  
  int found = 0;
  for (int i = 0; i < numSamples; i++) {
    float wind = parseJSONValue(cursor, "\"wind_speed_10m\"");
    if (wind >= 0) {
      char *next = strstr(cursor, "\"wind_speed_10m\"");
      if (next) {
        float dir = parseJSONValue(next, "\"wind_direction_10m\"");
        float wave = wind / 25.0f;
        printf("[SMART DATA]  Waypt %d (%.2f, %.2f): Wind %.1f km/h | Wave %.1f m\n", 
               i+1, lats[i], lons[i], wind, wave);
        applyRealWeather(lats[i], lons[i], wind, dir, wave);
        cursor = next + 20;
        found++;
      } else { cursor += 20; }
    } else {
        char *nextObj = strstr(cursor + 1, "{");
        if (nextObj) cursor = nextObj;
        else cursor += 20;
    }
  }
  printf("[SMART ROUTE] Verified %d/%d path locations.\n", found, numSamples);
  
  weatherThreadActive = 0; // Signal completion
  return 0;
}

// --- Geolocation removed: start point is set manually by the user ---

// --- Weather Implementation ---
void updateWeatherSimulation() {
  if (!weatherGrid)
    return;

  // 1. Initialize entire ocean with calm baselines (5 to 15 knots)
  for (int r = 0; r < gridH; r++) {
    for (int c = 0; c < gridW; c++) {
      float noise = (float)(rand() % 100) / 100.0f;
      weatherGrid[r * gridW + c].windSpeed = 5.0f + noise * 10.0f;
      weatherGrid[r * gridW + c].windDir = (float)(rand() % 360);
      weatherGrid[r * gridW + c].waveHeight = 0.5f + noise;
    }
  }

  // 2. Spawn fully random isolated storm clusters
  int numStorms =
      20 + (rand() % 20); // 20 to 40 random localized storms globally
  for (int i = 0; i < numStorms; i++) {
    // Pick a random center
    int cx = rand() % gridW;
    int cy = rand() % gridH;

    // Only spawn if it's over the ocean
    if (collisionGrid[cy * gridW + cx] == 1)
      continue;

    // Random storm variables
    float maxWind = 40.0f + (rand() % 40); // 40 to 80 peak wind
    int radius = 8 + (rand() % 15);       // Smaller storm size
    float baseDir = (float)(rand() % 360);

    // Apply storm influence outward from center
    int startY = (cy - radius < 0) ? 0 : cy - radius;
    int endY = (cy + radius >= gridH) ? gridH - 1 : cy + radius;
    int startX = (cx - radius < 0) ? 0 : cx - radius;
    int endX = (cx + radius >= gridW) ? gridW - 1 : cx + radius;

    for (int r = startY; r <= endY; r++) {
      for (int c = startX; c <= endX; c++) {
        if (collisionGrid[r * gridW + c] == 1)
          continue; // Skip land within radius

        float dist = sqrtf((r - cy) * (r - cy) + (c - cx) * (c - cx));
        if (dist <= radius) {
          // Intensity falls off from center to edge (bezier-like curve for
          // organic look)
          float intensity = 1.0f - (dist / radius);
          intensity = intensity * intensity;

          float addedWind = maxWind * intensity +
                            ((rand() % 10) / 10.0f * 5.0f); // Add organic noise

          if (weatherGrid[r * gridW + c].windSpeed < addedWind) {
            weatherGrid[r * gridW + c].windSpeed = addedWind;
            weatherGrid[r * gridW + c].waveHeight = 1.0f + (addedWind / 15.0f);

            // Cyclonic wind direction curve (rotates around center)
            float angleToCenter = atan2f(cy - r, cx - c) * 180.0f / 3.14159f;
            weatherGrid[r * gridW + c].windDir =
                fmodf(angleToCenter + 90.0f + baseDir, 360.0f);
          }
        }
      }
    }
  }
}

// --- Logic ---
void createCollisionGrid(SDL_Surface *surf) {
  gridW = surf->w / GRID_SCALE;
  gridH = surf->h / GRID_SCALE;
  collisionGrid = (unsigned char *)malloc(gridW * gridH);
  weatherGrid = (WeatherCell *)calloc(gridW * gridH, sizeof(WeatherCell));

  Uint32 *pixels = (Uint32 *)surf->pixels;
  for (int y = 0; y < gridH; y++) {
    for (int x = 0; x < gridW; x++) {
      Uint32 pixel = pixels[(y * GRID_SCALE * surf->w) + (x * GRID_SCALE)];
      Uint8 r, g, b;
      SDL_GetRGB(pixel, surf->format, &r, &g, &b);
      // Land is typically very dark grey/black (< 20).
      // Everything else should be treated as Ocean (0) to include all ocean
      // shades.
      collisionGrid[y * gridW + x] = (r < 20 && g < 20 && b < 20) ? 1 : 0;
    }
  }

  // Create padding (safety buffer) around land mass
  int paddingRadius = 4; // Width of the padding
  // Create temp grid to avoid cascading
  unsigned char *tempGrid = (unsigned char *)malloc(gridW * gridH);
  memcpy(tempGrid, collisionGrid, gridW * gridH);

  for (int y = 0; y < gridH; y++) {
    for (int x = 0; x < gridW; x++) {
      if (tempGrid[y * gridW + x] == 0) {
        // Check surrounding cells within radius for land (1)
        int foundLand = 0;
        for (int dy = -paddingRadius; dy <= paddingRadius; dy++) {
          for (int dx = -paddingRadius; dx <= paddingRadius; dx++) {
            // Circle check for smoother corners
            if (dx*dx + dy*dy <= paddingRadius*paddingRadius) {
               int ny = y + dy;
               int nx = x + dx;
               if (ny >= 0 && ny < gridH && nx >= 0 && nx < gridW) {
                 if (tempGrid[ny * gridW + nx] == 1) {
                   foundLand = 1;
                   break;
                 }
               }
            }
          }
          if (foundLand) break;
        }
        if (foundLand) {
           collisionGrid[y * gridW + x] = 2; // Mark as padded danger zone
        }
      }
    }
  }
  free(tempGrid);

  updateWeatherSimulation();
}

void snapToWater(Point *p) {
  int c = (int)(p->x + mapWidth / 2) / GRID_SCALE;
  int r = (int)(p->y + mapHeight / 2) / GRID_SCALE;
  if (r >= 0 && r < gridH && c >= 0 && c < gridW &&
      collisionGrid[r * gridW + c] != 1)
    return;
  for (int radius = 1; radius < 25; radius++) {
    for (int dr = -radius; dr <= radius; dr++) {
      for (int dc = -radius; dc <= radius; dc++) {
        int nr = r + dr, nc = c + dc;
        if (nr >= 0 && nr < gridH && nc >= 0 && nc < gridW &&
            collisionGrid[nr * gridW + nc] != 1) {
          p->x = (nc * GRID_SCALE) - mapWidth / 2.0f + (GRID_SCALE / 2.0f);
          p->y = (nr * GRID_SCALE) - mapHeight / 2.0f + (GRID_SCALE / 2.0f);
          return;
        }
      }
    }
  }
}

void calculateRouteStats() {
  if (!finalPath || pathLen == 0) {
    routeDistance = 0.0f;
    routeETA = 0.0f;
    routeFuel = 0.0f;
    routeRisk = 0.0f;
    // If we have no path, it's always STANDBY
    strcpy(routeStatus, "STANDBY");
    return;
  }

  // Calculate total distance in km
  routeDistance = 0.0f;
  float maxRisk = 0.0f;

  for (int i = 0; i < pathLen - 1; i++) {
    float wx1 = finalPath[i].c * GRID_SCALE - mapWidth / 2.0f;
    float wy1 = finalPath[i].r * GRID_SCALE - mapHeight / 2.0f;
    float wx2 = finalPath[i + 1].c * GRID_SCALE - mapWidth / 2.0f;
    float wy2 = finalPath[i + 1].r * GRID_SCALE - mapHeight / 2.0f;

    // Convert to lat/lon for accurate distance
    float lat1 = pixelToLat(worldToPixelY(wy1));
    float lon1 = pixelToLon(worldToPixelX(wx1));
    float lat2 = pixelToLat(worldToPixelY(wy2));
    float lon2 = pixelToLon(worldToPixelX(wx2));

    // Haversine formula for distance
    float dLat = (lat2 - lat1) * M_PI / 180.0f;
    float dLon = (lon2 - lon1) * M_PI / 180.0f;
    float a = sinf(dLat / 2) * sinf(dLat / 2) +
              cosf(lat1 * M_PI / 180.0f) * cosf(lat2 * M_PI / 180.0f) *
                  sinf(dLon / 2) * sinf(dLon / 2);
    float c_val = 2 * atan2f(sqrtf(a), sqrtf(1 - a));
    float legDist = 6371.0f * c_val;
    routeDistance += legDist;
    pathDistances[i+1] = routeDistance;

    // Track max risk along route
    float wind = weatherGrid[finalPath[i].r * gridW + finalPath[i].c].windSpeed;
    if (wind > STORM_THRESHOLD) {
      float risk = (wind / 100.0f) * 100.0f;
      if (risk > maxRisk)
        maxRisk = risk;
    }
  }

  // Calculate ETA in days (distance / speed, convert knots to km/day)
  float speedKmPerDay = shipSpeedKnots * 1.852f * 24.0f; // knots to km/day
  routeETA = routeDistance / speedKmPerDay;

  // Estimate fuel consumption (rough estimate: ~0.5 tons per 100km for
  // cargo ship)
  routeFuel = routeDistance * 0.5f / 100.0f;

  // Risk percentage
  routeRisk = maxRisk;
  if (routeRisk > 100.0f)
    routeRisk = 100.0f;

  if (smartRouteStage > 0 || pendingVerification) {
      strcpy(routeStatus, "PENDING");
  } else {
      strcpy(routeStatus, "OPTIMIZED");
  }
}

int astar() {
  if (!p1.valid || !p2.valid)
    return 0;
  snapToWater(&p1);
  snapToWater(&p2);
  int startC = (int)(p1.x + mapWidth / 2) / GRID_SCALE;
  int startR = (int)(p1.y + mapHeight / 2) / GRID_SCALE;
  int endC = (int)(p2.x + mapWidth / 2) / GRID_SCALE;
  int endR = (int)(p2.y + mapHeight / 2) / GRID_SCALE;

  MinHeap openList = {malloc(sizeof(Node *) * gridW * gridH), 0};
  Node *nodes = (Node *)calloc(gridW * gridH, sizeof(Node));
  float *gScore = (float *)malloc(gridW * gridH * sizeof(float));
  for (int i = 0; i < gridW * gridH; i++)
    gScore[i] = 1e9f;

  int startIdx = startR * gridW + startC;
  gScore[startIdx] = 0;
  nodes[startIdx].pos = (GridPos){startR, startC};
  nodes[startIdx].f = sqrtf(pow(startR - endR, 2) + pow(startC - endC, 2));
  pushHeap(&openList, &nodes[startIdx]);

  int found = 0;
  while (openList.size > 0) {
    Node *curr = popHeap(&openList);
    if (curr->pos.r == endR && curr->pos.c == endC) {
      found = 1;
      pathLen = 0;
      Node *temp = curr;
      while (temp) {
        pathLen++;
        temp = temp->parent;
      }
      if (finalPath)
        free(finalPath);
      finalPath = malloc(sizeof(GridPos) * pathLen);
      temp = curr;
      for (int i = pathLen - 1; i >= 0; i--) {
        finalPath[i] = temp->pos;
        temp = temp->parent;
      }
      break;
    }

    for (int dr = -1; dr <= 1; dr++) {
      for (int dc = -1; dc <= 1; dc++) {
        if (dr == 0 && dc == 0)
          continue;
        int nr = curr->pos.r + dr, nc = curr->pos.c + dc;
        if (nr < 0 || nr >= gridH || nc < 0 || nc >= gridW ||
            collisionGrid[nr * gridW + nc] == 1)
          continue;

        float stepCost = (dr == 0 || dc == 0) ? 1.0f : 1.414f;
        if (collisionGrid[nr * gridW + nc] == 2)
          stepCost += PADDING_COST;

        // --- WEATHER PENALTY ---
        float wind = weatherGrid[nr * gridW + nc].windSpeed;
        if (wind > STORM_THRESHOLD) {
          stepCost +=
              (wind * 8.0f); // Penalize storms to force routing around them
        }

        float tentativeG = gScore[curr->pos.r * gridW + curr->pos.c] + stepCost;
        if (tentativeG < gScore[nr * gridW + nc]) {
          int idx = nr * gridW + nc;
          gScore[idx] = tentativeG;
          nodes[idx].pos = (GridPos){nr, nc};
          nodes[idx].parent = curr;
          nodes[idx].g = tentativeG;
          nodes[idx].h = sqrtf(pow(nr - endR, 2) + pow(nc - endC, 2)) * 1.2f;
          nodes[idx].f = nodes[idx].g + nodes[idx].h;
          pushHeap(&openList, &nodes[idx]);
        }
      }
    }
  }
  free(openList.nodes);
  free(nodes);
  free(gScore);
  // No longer needed: snprintf(infoText, sizeof(infoText), found ? "Route Calculated (Storms Avoided)" : "No Route Possible");
  calculateRouteStats();
  return found;
}

void getLocation(float *lat, float *lon) {
    const char *tmpFile = "loc_tmp.txt";
#ifdef _WIN32
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "powershell.exe -ExecutionPolicy Bypass -File get_location.ps1 > %s 2>$null", tmpFile);
#else
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "curl -s \"http://ip-api.com/line?fields=lat,lon\" > %s", tmpFile);
#endif

    if (system(cmd) != 0) {
#ifdef _WIN32
        printf("[LOC ERROR] Geolocation failed. Opening settings...\n");
        system("start ms-settings:privacy-location");
#else
        printf("[LOC ERROR] Geolocation failed.\n");
#endif
        return;
    }

    FILE *f = fopen(tmpFile, "r");
    if (f) {
#ifdef _WIN32
        if (fscanf(f, "%f, %f", lat, lon) == 2) {
            printf("[LOC] Windows Location: %.4f, %.4f\n", *lat, *lon);
        }
#else
        float l1, l2;
        if (fscanf(f, "%f\n%f", &l1, &l2) == 2) {
            *lat = l1; *lon = l2;
            printf("[LOC] IP-Based Location: %.4f, %.4f\n", *lat, *lon);
        }
#endif
        fclose(f);
        remove(tmpFile);
    }
}

void triggerStartJourney() {
    float lat = -255.0f, lon = -255.0f;
    getLocation(&lat, &lon);

    if (lat < -180.0f) return; // Failure

    p1.x = lonToPixelX(lon) - mapWidth / 2.0f;
    p1.y = latToPixelY(lat) - mapHeight / 2.0f;
    p1.valid = 1;
    p1.alpha = 255.0f;
    snapToWater(&p1);
    
    snprintf(startCoordTxt, sizeof(startCoordTxt), "%.4f, %.4f", lat, lon);
    
    if (p1.valid && p2.valid) {
        astar();
    }
}

int main(int argc, char *argv[]) {
  // Keep terminal live: disable stdout/stderr buffering
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);

  printf("========================================\n");
  printf("   Sea Route Planner - Storm Avoidance  \n");
  printf("========================================\n");
  printf("[INIT] Application starting...\n");

  // Initialize departure time with real system time
  time_t now = time(NULL);
  struct tm *t_now = localtime(&now);
  strftime(depDateTxt, sizeof(depDateTxt), "%Y-%m-%d %H:%M", t_now);

  srand((unsigned int)(now / 21600)); // Consistent weather for 6 hours
  SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO);

  SDL_DisplayMode dm;
  if (SDL_GetCurrentDisplayMode(0, &dm) == 0) {
    winWidth = (int)(dm.w * 0.9f);
    winHeight = (int)(dm.h * 0.9f);
  }
  TTF_Init();

  SDL_Window *win =
      SDL_CreateWindow("Sea Route Planner - Storm Avoidance",
                       SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, winWidth,
                       winHeight + TOPBAR, SDL_WINDOW_RESIZABLE);
  SDL_Renderer *ren = SDL_CreateRenderer(
      win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

  // --- Loading screen font (loaded first so we can show progress) ---
  TTF_Font *font = TTF_OpenFont("assets/fonts/DejaVuSans.ttf", 16);
  TTF_Font *smallFont = TTF_OpenFont("assets/fonts/DejaVuSans.ttf", 12);
  TTF_Font *statusFont = TTF_OpenFont("assets/fonts/DejaVuSans.ttf", 17);
  TTF_Font *valueFont =
      TTF_OpenFont("assets/fonts/JetBrainsMono-ExtraBold.ttf", 18);
  TTF_Font *labelFont = TTF_OpenFont("assets/fonts/Inter_18pt-Regular.ttf", 14);
  TTF_Font *titleFont = TTF_OpenFont("assets/fonts/Inter_18pt-Regular.ttf", 16);
  TTF_Font *aegisFont =
      TTF_OpenFont("assets/fonts/JetBrainsMono-ExtraBold.ttf", 28);

  if (!font || !smallFont || !statusFont || !valueFont || !labelFont || !titleFont || !aegisFont) {
    printf("Font error: %s\n", TTF_GetError());
  } else {
    TTF_SetFontHinting(font, TTF_HINTING_LIGHT);
    TTF_SetFontHinting(smallFont, TTF_HINTING_LIGHT);
    TTF_SetFontHinting(statusFont, TTF_HINTING_LIGHT);
    TTF_SetFontHinting(valueFont, TTF_HINTING_LIGHT);
    TTF_SetFontHinting(labelFont, TTF_HINTING_LIGHT);
    TTF_SetFontHinting(titleFont, TTF_HINTING_LIGHT);
    TTF_SetFontHinting(aegisFont, TTF_HINTING_LIGHT);
  }

  // --- Loading Steps ---
  drawLoadingScreen(ren, font, "Initializing audio system...", 1, 7);
  // Load Audio
  SDL_AudioSpec wavSpec;
  if (SDL_LoadWAV("assets/tick.wav", &wavSpec, &tickAudioBuf, &tickAudioLen) == NULL) {
      printf("[SDL ERROR] Failed to load assets/tick.wav: %s\n", SDL_GetError());
  } else {
      audioDevice = SDL_OpenAudioDevice(NULL, 0, &wavSpec, NULL, 0);
      SDL_PauseAudioDevice(audioDevice, 0);
  }

  drawLoadingScreen(ren, font, "Loading UI assets...", 2, 7);
  startTex = LoadTexture(ren, "assets/start.png");
  endTex = LoadTexture(ren, "assets/end.png"); 
  
  // Load Icons
  iconShip     = LoadTexture(ren, "assets/icons/ship.png");
  iconApi      = LoadTexture(ren, "assets/icons/api.png");
  iconCloud    = LoadTexture(ren, "assets/icons/cloud.png");
  iconKeyboard = LoadTexture(ren, "assets/icons/keyboard.png");
  iconGraphOn  = LoadTexture(ren, "assets/icons/graph_on.png");
  iconGraphOff = LoadTexture(ren, "assets/icons/graph_off.png");
  iconOnline   = LoadTexture(ren, "assets/icons/online.png");
  iconUnplug   = LoadTexture(ren, "assets/icons/unplug.png");
  iconSettings = LoadTexture(ren, "assets/icons/settings.png");
  iconLoader   = LoadTexture(ren, "assets/icons/loader-circle.png");
  iconArrow    = LoadTexture(ren, "assets/icons/arrow.png");

  drawLoadingScreen(ren, font, "Loading map texture...", 3, 7);
  SDL_Surface *surf = LoadSurface("assets/temp1.png");
  mapWidth = surf->w;
  mapHeight = surf->h;

  drawLoadingScreen(ren, font, "Building ocean collision grid...", 4, 7);
  createCollisionGrid(surf);

  drawLoadingScreen(ren, font, "Uploading map to GPU...", 5, 7);
  mapTex = SDL_CreateTextureFromSurface(ren, surf);
  SDL_FreeSurface(surf);

  drawLoadingScreen(ren, font, "Scanning global marine weather (API)...", 6, 7);
  scanOceanWeather();

  drawLoadingScreen(ren, font, "Ready!", 7, 7);
  SDL_Delay(400); // Brief pause so user can see "100%" complete

  // Initial camera centering at pixel (5600, 3400)
  camX = 5600.0f - mapWidth / 2.0f;
  camY = 3400.0f - mapHeight / 2.0f;

  int dragging = 0, lastMouseX = 0, lastMouseY = 0, running = 1;
  int potentialDrag = 0;              // Track if left click might become a drag
  int startDragX = 0, startDragY = 0; // Origin of click
  Uint32 lastTicks = SDL_GetTicks();
  Uint32 lastZoomSound = 0;
  SDL_Rect stormBtn = {10, 180, 300,
                       38}; // Storm toggle button (below marine panel)

  while (running) {
    Uint32 currentTicks = SDL_GetTicks();
    float deltaTime = (currentTicks - lastTicks) / 1000.0f;
    if (deltaTime > 0.1f)
      deltaTime = 0.016f;
    lastTicks = currentTicks;

    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_QUIT)
        running = 0;
      if (e.type == SDL_WINDOWEVENT &&
          e.window.event == SDL_WINDOWEVENT_RESIZED) {
        winWidth = e.window.data1;
        winHeight = e.window.data2;
      }
      if (e.type == SDL_TEXTINPUT) {
         char *targetBuf = NULL;
         int maxLen = 0;
         if (showConfigWindow && configActiveTab == 0) {
             if (activeInputIndex == 0) { targetBuf = inputShipName; maxLen = 63; }
             else if (activeInputIndex == 1) { targetBuf = inputMaxSpeed; maxLen = 15; }
             else if (activeInputIndex == 2) { targetBuf = inputCruiseSpeed; maxLen = 15; }
             else if (activeInputIndex == 3) { targetBuf = inputFuelRate; maxLen = 15; }
             else if (activeInputIndex == 4) { targetBuf = inputWaveTol; maxLen = 15; }
         } else if (showJourneyPanel) {
             if (activeInputIndex == 5) { targetBuf = startCoordTxt; maxLen = 63; }
             else if (activeInputIndex == 6) { targetBuf = endCoordTxt; maxLen = 63; }
             else if (activeInputIndex == 7) { targetBuf = depDateTxt; maxLen = 31; }
         }
         
         if (targetBuf) {
            int curLen = strlen(targetBuf);
            if (curLen + strlen(e.text.text) < maxLen) {
                strcat(targetBuf, e.text.text);
            }
         }
      }

      if (e.type == SDL_KEYDOWN) {
        if ((showConfigWindow || showJourneyPanel) && activeInputIndex != -1) {
             char *targetBuf = NULL;
             if (activeInputIndex == 0) targetBuf = inputShipName;
             else if (activeInputIndex == 1) targetBuf = inputMaxSpeed;
             else if (activeInputIndex == 2) targetBuf = inputCruiseSpeed;
             else if (activeInputIndex == 3) targetBuf = inputFuelRate;
             else if (activeInputIndex == 4) targetBuf = inputWaveTol;
             else if (activeInputIndex == 5) targetBuf = startCoordTxt;
             else if (activeInputIndex == 6) targetBuf = endCoordTxt;
             else if (activeInputIndex == 7) targetBuf = depDateTxt;
             
             if (targetBuf && e.key.keysym.sym == SDLK_BACKSPACE && strlen(targetBuf) > 0) {
                 targetBuf[strlen(targetBuf) - 1] = '\0';
             }
             if (e.key.keysym.sym == SDLK_RETURN || e.key.keysym.sym == SDLK_RETURN2 || e.key.keysym.sym == SDLK_KP_ENTER) {
                 // Try parsing coordinates if it was a coord field
                 if (activeInputIndex == 5 || activeInputIndex == 6) {
                     float lat = 0, lon = 0;
                     if (sscanf(targetBuf, "%f, %f", &lat, &lon) == 2) {
                         Point *targetP = (activeInputIndex == 5) ? &p1 : &p2;
                         targetP->x = lonToPixelX(lon) - mapWidth / 2.0f;
                         targetP->y = latToPixelY(lat) - mapHeight / 2.0f;
                         targetP->valid = 1;
                         targetP->alpha = 255.0f;
                         snapToWater(targetP);
                         if (p1.valid && p2.valid) astar();
                     }
                 }
                 activeInputIndex = -1; // Deselect on enter
                 SDL_StopTextInput();
             }
             continue; // absorb key when editing
        }

        float moveSpeed = 500.0f / zoom; // Adjust speed by zoom
        switch (e.key.keysym.sym) {
        case SDLK_w:
        case SDLK_UP:
          velY = -moveSpeed;
          break;
        case SDLK_s:
        case SDLK_DOWN:
          velY = moveSpeed;
          break;
        case SDLK_a:
        case SDLK_LEFT:
          velX = -moveSpeed;
          break;
        case SDLK_d:
        case SDLK_RIGHT:
          velX = moveSpeed;
          break;
        case SDLK_u: {
          // Update weather for Points A and B
          if (p1.valid) {
            float px = worldToPixelX(p1.x);
            float py = worldToPixelY(p1.y);
            fetchWeatherAndApply(pixelToLat(py), pixelToLon(px));
          }
          if (p2.valid) {
            float px = worldToPixelX(p2.x);
            float py = worldToPixelY(p2.y);
            fetchWeatherAndApply(pixelToLat(py), pixelToLon(px));
          }
          // Re-calculate path if both exist
          if (p1.valid && p2.valid) {
            astar();
          }
        } break;
        case SDLK_f: // F to fix camera reset manually if lost
          camX = 5600.0f - mapWidth / 2.0f;
          camY = 3400.0f - mapHeight / 2.0f;
          zoom = 1.0f;
          targetZoom = 1.0f;
          velX = 0;
          velY = 0;
          break;

        case SDLK_k:
          scanOceanWeather();
          if (p1.valid && p2.valid)
            astar();
          break;
        }
      }
      if (e.type == SDL_MOUSEWHEEL) {
        int mx, my;
        SDL_GetMouseState(&mx, &my);

        // Store the world position under the mouse and the mouse screen
        // position
        zoomWorldX = screenToWorldX(mx);
        zoomWorldY = screenToWorldY(my);
        zoomMouseX = mx;
        zoomMouseY = my;

        targetZoom += e.wheel.y * 0.12f;
        if (targetZoom < 0.289f)
          targetZoom = 0.289f;
        if (targetZoom > 2.0f)
          targetZoom = 2.0f; // Limit zoom out
        if (tickAudioBuf && (currentTicks - lastZoomSound > 100)) {
          SDL_ClearQueuedAudio(audioDevice);
          SDL_QueueAudio(audioDevice, tickAudioBuf, tickAudioLen);
          lastZoomSound = currentTicks;
        }
      }
      if (e.type == SDL_MOUSEBUTTONDOWN) {
        int mx = e.button.x, my = e.button.y;
        
        // Handle "Plan Journey" Toggle Button (Bottom Left)
        if (!showJourneyPanel && !showConfigWindow) {
            if (mx >= 20 && mx <= 220 && my >= winHeight - 110 && my <= winHeight - 65) {
                showJourneyPanel = 1;
                continue; // block map drag
            }
        }
        
        // Handle Sidebar interaction
        if (showJourneyPanel && mx < 320) {
            // Close Button
            if (mx >= 320 - 45 && mx <= 320 - 10 && my >= 15 && my <= 45) {
                showJourneyPanel = 0;
                activeInputIndex = -1;
                SDL_StopTextInput();
            }
            // Inputs
            int curY = 125; // Matching sBox.y
            if (mx >= 30 && mx <= 290 && my >= curY && my <= curY + 35) activeInputIndex = 5; // Start point field
            curY += 85; 
            if (mx >= 30 && mx <= 290 && my >= curY && my <= curY + 35) activeInputIndex = 6; // End point field
            curY += 85;
            if (mx >= 30 && mx <= 290 && my >= curY && my <= curY + 35) activeInputIndex = 7; // Date field
            
            if (activeInputIndex >= 5) SDL_StartTextInput();
            
            // Start Journey Button
            int sjY = 125 + 85 + 85 + 70; // Matching sjBtn.y
            if (mx >= 30 && mx <= 290 && my >= sjY && my <= sjY + 45) {
                triggerStartJourney();
            }

            // Reset Button
            curY = sjY + 65;
            if (mx >= 30 && mx <= 290 && my >= curY && my <= curY + 40) {
                p1.valid = 0; p2.valid = 0;
                if (finalPath) { free(finalPath); finalPath = NULL; }
                pathLen = 0;
                calculateRouteStats();
            }
            continue; // block map drag
        }

        if (e.button.button == SDL_BUTTON_LEFT) {
          if (showConfigWindow) {
            // Handle clicks in modal
            int mx = e.button.x, my = e.button.y;
            // The winWidth/winHeight vars are already present from earlier
            int modalW = 800, modalH = 500;
            int modalX = winWidth / 2 - modalW / 2;
            int modalY = winHeight / 2 - modalH / 2;

            // Close button (X)
            if (mx >= modalX + modalW - 40 && mx <= modalX + modalW - 10 &&
                my >= modalY + 10 && my <= modalY + 40) {
              showConfigWindow = 0;
              SDL_StopTextInput();
              activeInputIndex = -1;
            }

            // Tabs
            if (mx >= modalX && mx <= modalX + 220) {
              int tabStartItemY = modalY + 80;
              if (my >= tabStartItemY && my <= tabStartItemY + 40) configActiveTab = 0;
              else if (my >= tabStartItemY + 50 && my <= tabStartItemY + 90) configActiveTab = 1;
              else if (my >= tabStartItemY + 100 && my <= tabStartItemY + 140) configActiveTab = 2;
              else if (my >= tabStartItemY + 150 && my <= tabStartItemY + 190) configActiveTab = 3;
              activeInputIndex = -1; // clear input focus
            }

            // Input fields selection in Tab 0
            if (configActiveTab == 0) {
               int cx = modalX + 250;
               int cy = modalY + 90 + 40; // Starts after title
               // Bounding box logic matches drawBox offsets
               // Reset before check
               int prevInput = activeInputIndex;
               activeInputIndex = -1;
               
               if (mx >= cx && mx <= cx + 500 && my >= cy + 25 && my <= cy + 60) activeInputIndex = 0; cy += 80;
               if (mx >= cx && mx <= cx + 230 && my >= cy + 25 && my <= cy + 60) activeInputIndex = 1;
               if (mx >= cx + 270 && mx <= cx + 500 && my >= cy + 25 && my <= cy + 60) activeInputIndex = 2; cy += 80;
               if (mx >= cx && mx <= cx + 230 && my >= cy + 25 && my <= cy + 60) activeInputIndex = 3;
               if (mx >= cx + 270 && mx <= cx + 500 && my >= cy + 25 && my <= cy + 60) activeInputIndex = 4;
               
                if (activeInputIndex != prevInput) {
                   if (activeInputIndex != -1) SDL_StartTextInput();
                   else SDL_StopTextInput();
                }
            } else if (configActiveTab == 1) {
                // Clear Cache Button Logic
                int cx = modalX + 250;
                int cy = modalY + 90 + 60 + 100;
                if (mx >= cx && mx <= cx + 260 && my >= cy && my <= cy + 45) {
                    if (weatherGrid) {
                        for (int i = 0; i < gridW * gridH; i++) {
                            weatherGrid[i].windSpeed = 0;
                            weatherGrid[i].waveHeight = 0;
                            weatherGrid[i].windDir = 0;
                        }
                    }
                    remove("path_weather_data.json");
                    remove("marine_scan_data.json");
                    printf("[CACHE] Weather data cleared.\n");
                }
            } else if (configActiveTab == 2) {
               // Weather Settings Toggles
               int cx = modalX + 250;
               int cy = modalY + 90 + 50;
               int tw = 510;
               
               if (mx >= cx + tw - 50 && mx <= cx + tw && my >= cy + 5 && my <= cy + 27) showStorms = !showStorms; cy += 80;
               if (mx >= cx + tw - 50 && mx <= cx + tw && my >= cy + 5 && my <= cy + 27) showWindParticles = !showWindParticles; cy += 80;
               if (mx >= cx + tw - 50 && mx <= cx + tw && my >= cy + 5 && my <= cy + 27) showWaveHeights = !showWaveHeights;
            }
            
            // Buttons at bottom right
            int btnY = modalY + modalH - 50;
            if (my >= btnY - 10 && my <= btnY + 40) {
              if (mx >= modalX + modalW - 250 && mx <= modalX + modalW - 190) { // Cancel
                 showConfigWindow = 0;
                 SDL_StopTextInput();
                 activeInputIndex = -1;
              } else if (mx >= modalX + modalW - 180 && mx <= modalX + modalW - 20) { // Apply
                 // Handle Apply Settings!
                 strncpy(shipName, inputShipName, sizeof(shipName) - 1);
                 shipSpeedKnots = atof(inputCruiseSpeed);
                 snprintf(shipSpeed, sizeof(shipSpeed), "%.1f kts", shipSpeedKnots);
                 fuelRate = atof(inputFuelRate);
                 maxWaveTol = atof(inputWaveTol);
                 
                 showConfigWindow = 0;
                 SDL_StopTextInput();
                 activeInputIndex = -1;
                 
                 if (p1.valid && p2.valid) astar(); // Recompute routing with new vessel params
              }
            }
            continue; // block map interaction
          }

          // Start potential drag or click
          potentialDrag = 1;
          startDragX = e.button.x;
          startDragY = e.button.y;
          lastMouseX = e.button.x;
          lastMouseY = e.button.y;
          velX = velY = 0;
          for (int i = 0; i < VELOCITY_SAMPLES; i++) {
            frameVelX[i] = 0;
            frameVelY[i] = 0;
          }
        } else if (e.button.button == SDL_BUTTON_RIGHT) {
          // Explicit drag
          dragging = 1;
          lastMouseX = e.button.x;
          lastMouseY = e.button.y;
          velX = velY = 0;
          for (int i = 0; i < VELOCITY_SAMPLES; i++) {
            frameVelX[i] = 0;
            frameVelY[i] = 0;
          }
        }
      }

      if (e.type == SDL_MOUSEBUTTONUP) {
        if (e.button.button == SDL_BUTTON_LEFT) {
          if (dragging) {
            // End drag
            dragging = 0;
            float avgX = 0, avgY = 0;
            for (int i = 0; i < VELOCITY_SAMPLES; i++) {
              avgX += frameVelX[i];
              avgY += frameVelY[i];
            }
            velX = avgX / VELOCITY_SAMPLES;
            velY = avgY / VELOCITY_SAMPLES;
          } else if (potentialDrag) {
            if (showConfigWindow) {
              potentialDrag = 0;
              continue;
            }
            // Check config toggle button first
            // Button is at top-center, roughly width 220, height 30
            int configBtnX = winWidth / 2 - 110;
            int configBtnY = 15;
            int bx = e.button.x, by = e.button.y;

            if (bx >= configBtnX && bx <= configBtnX + 220 &&
                by >= configBtnY && by <= configBtnY + 30) {
              showConfigWindow = 1;
            }
            // Check storm toggle button
            else if (bx >= stormBtn.x && bx <= stormBtn.x + stormBtn.w &&
                by >= stormBtn.y && by <= stormBtn.y + stormBtn.h) {
              showStorms = !showStorms;
            } else {
              // It was just a click on the map! Set Waypoint A/B
              float wx = screenToWorldX(e.button.x),
                    wy = screenToWorldY(e.button.y);
              
              if (!p1.valid) {
                // First click: Set Point A
                p1 = (Point){wx, wy, 1, 0.0f};
                p2.valid = 0;
                pathLen = 0;
                if (finalPath) { free(finalPath); finalPath = NULL; }
                
                // Auto-sync coordinates to Sidebar
                snprintf(startCoordTxt, sizeof(startCoordTxt), "%.3f, %.3f", 
                         pixelToLat(worldToPixelY(wy)), pixelToLon(worldToPixelX(wx)));
                
                calculateRouteStats(); // Set to STANDBY
                // snprintf(infoText, ...) removed
              } else if (!p2.valid) {
                // Second click: Set Point B and calculate draft route
                p2 = (Point){wx, wy, 1, 0.0f};
                
                // Auto-sync coordinates to Sidebar
                snprintf(endCoordTxt, sizeof(endCoordTxt), "%.3f, %.3f", 
                         pixelToLat(worldToPixelY(wy)), pixelToLon(worldToPixelX(wx)));
                         
                if (astar()) {
                    // Initial Pass success: now trigger safety verification scan
                    pendingVerification = 1;
                    smartRouteStage = 1; 
                    pathDrawProgress = 0.0f;
                    smartRouteAnim = 0.0f;
                }
              } else {
                // Third click: Clear both and reset
                p1.valid = 0;
                p2.valid = 0;
                pathLen = 0;
                pathDrawProgress = 0.0f;
                spinnerTimer = 0.0f;
                pendingVerification = 0;
                if (finalPath) { free(finalPath); finalPath = NULL; }
                
                // Reset Sidebar strings
                strncpy(startCoordTxt, "Not Set", sizeof(startCoordTxt));
                strncpy(endCoordTxt, "Not Set", sizeof(endCoordTxt));
                
                calculateRouteStats(); // Set to STANDBY
                // snprintf(infoText, ...) removed
              }
            }
          }
          potentialDrag = 0;
        } else if (e.button.button == SDL_BUTTON_RIGHT) {
          dragging = 0;
          potentialDrag = 0;
          float avgX = 0, avgY = 0;
          for (int i = 0; i < VELOCITY_SAMPLES; i++) {
            avgX += frameVelX[i];
            avgY += frameVelY[i];
          }
          velX = avgX / VELOCITY_SAMPLES;
          velY = avgY / VELOCITY_SAMPLES;
        }
      }

      if (e.type == SDL_MOUSEMOTION) {
        mouseX = e.motion.x;
        mouseY = e.motion.y;
        isHoveringPath = 0;

        if (!showConfigWindow && !dragging && finalPath) {
            float mwx = screenToWorldX(mouseX);
            float mwy = screenToWorldY(mouseY);
            for (int i = 0; i < pathLen - 1; i++) {
                float wx1 = finalPath[i].c * GRID_SCALE - mapWidth / 2.0f;
                float wy1 = finalPath[i].r * GRID_SCALE - mapHeight / 2.0f;
                float wx2 = finalPath[i+1].c * GRID_SCALE - mapWidth / 2.0f;
                float wy2 = finalPath[i+1].r * GRID_SCALE - mapHeight / 2.0f;
                
                if (fabs(wx1 - wx2) < mapWidth / 2.0f) {
                   float d = distPointToSegment(mwx, mwy, wx1, wy1, wx2, wy2);
                   if (d < (10.0f / zoom)) { // 10 pixel radius
                       isHoveringPath = 1;
                       hoverWind = weatherGrid[finalPath[i].r * gridW + finalPath[i].c].windSpeed;
                       hoverWave = weatherGrid[finalPath[i].r * gridW + finalPath[i].c].waveHeight;
                       hoverDir  = weatherGrid[finalPath[i].r * gridW + finalPath[i].c].windDir;
                       break;
                   }
                }
            }
        }

        if (showConfigWindow) continue;
        if (potentialDrag) {
          // Check if moved enough to count as drag
          int deltaX = abs(e.motion.x - startDragX);
          int deltaY = abs(e.motion.y - startDragY);
          if (deltaX > 5 || deltaY > 5) {
            dragging = 1;
            potentialDrag = 0; // Confirm it's a drag, not a click
          }
        }

        if (dragging) {
          float dx = (e.motion.x - lastMouseX) / zoom;
          float dy = (e.motion.y - lastMouseY) / zoom;
          camX -= dx;
          camY -= dy;
          frameVelX[velIdx] = -dx / deltaTime;
          frameVelY[velIdx] = -dy / deltaTime;
          velIdx = (velIdx + 1) % VELOCITY_SAMPLES;
          lastMouseX = e.motion.x;
          lastMouseY = e.motion.y;
          wrapCamera();
        }
      }
    }

    // Smooth zoom interpolation with camera adjustment to keep world
    // point under mouse
    float prevZoom = zoom;
    zoom += (targetZoom - zoom) * 0.12f;
    configAnimProgress += (showConfigWindow - configAnimProgress) * 0.15f;
    if (spinnerTimer > 0) {
        spinnerTimer -= deltaTime;
        loadingRotation += 2.0f;
    } 
    
    // Smart Routing State Machine
    if (smartRouteStage > 0) {
        smartRouteAnim += (1.0f - smartRouteAnim) * 0.15f;
        loadingRotation += 2.5f; 
        
        if (smartRouteStage == 1 && pathDrawProgress >= 1.0f) {
            // Stage 1: Wait for initial draw, then start thread
            if (!weatherThreadActive) {
                weatherThreadActive = 1;
                weatherThread = SDL_CreateThread(samplePathWeatherThread, "WeatherFetchThread", NULL);
                SDL_DetachThread(weatherThread);
                smartRouteStage = 2; // Transition to "Wait for API"
            }
        } else if (smartRouteStage == 2 && !weatherThreadActive) {
            // Stage 2: Thread finished, transition to Optimization
            smartRouteStage = 3; 
        } else if (smartRouteStage == 3) {
            // Stage 3: Perform A* optimization and cleanup
            astar();
            pathDrawProgress = 0.0f; // Re-draw the safety-optimized route
            smartRouteStage = 0;     // Modal will fade out
            pendingVerification = 0;
            calculateRouteStats();   // Re-calc once more to set OPTIMIZED status
        }
    } else {
        smartRouteAnim += (0.0f - smartRouteAnim) * 0.2f;
    }

    // Journey Panel Animation
    journeyPanelAnim += (showJourneyPanel - journeyPanelAnim) * 0.15f;

    // Interactive Path Tracking (When Journey Panel is open)
    pathHoverIndex = -1;
    if (showJourneyPanel && finalPath && pathLen > 1) {
        int mx, my;
        SDL_GetMouseState(&mx, &my);
        float mouseWX = screenToWorldX(mx);
        float mouseWY = screenToWorldY(my);
        
        float minDist = 1e9f;
        for (int i = 0; i < pathLen - 1; i++) {
            float x1 = finalPath[i].c * GRID_SCALE - mapWidth / 2.0f;
            float y1 = finalPath[i].r * GRID_SCALE - mapHeight / 2.0f;
            float x2 = finalPath[i+1].c * GRID_SCALE - mapWidth / 2.0f;
            float y2 = finalPath[i+1].r * GRID_SCALE - mapHeight / 2.0f;
            
            // Find closest point on line segment (x1,y1) -> (x2,y2) to (mouseWX, mouseWY)
            float dx = x2 - x1;
            float dy = y2 - y1;
            float t = ((mouseWX - x1) * dx + (mouseWY - y1) * dy) / (dx*dx + dy*dy);
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            
            float px = x1 + t * dx;
            float py = y1 + t * dy;
            float d = sqrtf((mouseWX - px) * (mouseWX - px) + (mouseWY - py) * (mouseWY - py));
            
            if (d < minDist && d < 100.0f / zoom) { // Only track if reasonably close
                minDist = d;
                pathHoverIndex = i;
                pathHoverT = t;
            }
        }
    }

    if (finalPath && pathDrawProgress < 1.0f) {
        pathDrawProgress += deltaTime * 2.5f;
        if (pathDrawProgress > 1.0f) pathDrawProgress = 1.0f;
    }

    // Adjust camera to keep zoomWorldX, zoomWorldY under the mouse cursor
    if (fabs(targetZoom - zoom) > 0.001f) {
      camX = zoomWorldX - (zoomMouseX - winWidth / 2.0f) / zoom;
      camY = zoomWorldY - (zoomMouseY - winHeight / 2.0f - TOPBAR) / zoom;
    }

    if (!dragging) {
      camX += velX * deltaTime;
      camY += velY * deltaTime;
      velX *= friction;
      velY *= friction;
      if (fabs(velX) < 1.0f)
        velX = 0;
      if (fabs(velY) < 1.0f)
        velY = 0;
      wrapCamera();
    }

    // Vertical Panning Constraints
    float limit = (mapHeight / 2.0f) - (winHeight / 2.0f) / zoom;
    if (limit < 0)
      camY = 0;
    else {
      if (camY < -limit)
        camY = -limit;
      if (camY > limit)
        camY = limit;
    }

    if (p1.valid && p1.alpha < 255)
      p1.alpha += 15;
    if (p2.valid && p2.alpha < 255)
      p2.alpha += 15;

    // Auto-refresh weather every 60s
    if (currentTicks - lastWeatherUpdate > 60000) {
      if (p1.valid || p2.valid) {
        printf("Auto-refreshing weather...\n");
        // Only refresh points if they exist
        if (p1.valid) {
          float px = worldToPixelX(p1.x);
          float py = worldToPixelY(p1.y);
          fetchWeatherAndApply(pixelToLat(py), pixelToLon(px));
        }
        if (p2.valid) {
          float px = worldToPixelX(p2.x);
          float py = worldToPixelY(p2.y);
          fetchWeatherAndApply(pixelToLat(py), pixelToLon(px));
        }
        // Update route
        if (p1.valid && p2.valid)
          astar();
      }
      lastWeatherUpdate = currentTicks;
    }

    // Rendering start: Get actual window size
    int currW, currH;
    SDL_GetRendererOutputSize(ren, &currW, &currH);

    SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
    SDL_RenderClear(ren);

    for (int dx = -1; dx <= 1; dx++) {
      SDL_Rect dst = {worldToScreenX(-mapWidth / 2 + dx * mapWidth),
                      worldToScreenY(-mapHeight / 2), (int)(mapWidth * zoom),
                      (int)(mapHeight * zoom)};
      SDL_RenderCopy(ren, mapTex, NULL, &dst);
    }

    // --- Render Weather Overlay ---
    if (showStorms) {
      SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
      for (int r = 0; r < gridH; r += 2) {
        for (int c = 0; c < gridW; c += 2) {
          if (collisionGrid[r * gridW + c] == 1)
            continue;
          float wind = weatherGrid[r * gridW + c].windSpeed;
          if (wind > STORM_THRESHOLD) {
            float wx = c * GRID_SCALE - mapWidth / 2.0f;
            float wy = r * GRID_SCALE - mapHeight / 2.0f;
            int screenX = worldToScreenX(wx);
            int screenY = worldToScreenY(wy);
            
            // Base size scales with wind intensity
            float sizeFactor = 1.0f + (wind - STORM_THRESHOLD) / 60.0f; // Scale from 1.0 to ~2.5
            int baseSize = (int)(GRID_SCALE * 2 * zoom * sizeFactor);

            // Render a "blob" using 3 overlapping circles/rects with fixed pseudo-random offsets
            SDL_Rect blobs[4];
            int offsetsX[] = {0, baseSize/4, -baseSize/5, baseSize/6};
            int offsetsY[] = {0, -baseSize/5, baseSize/4, baseSize/5};
            float scales[] = {1.0f, 0.85f, 0.75f, 0.9f};

            if (wind < 40.0f)
              SDL_SetRenderDrawColor(ren, 255, 200, 0, 50); // Yellow/Orange outer
            else
              SDL_SetRenderDrawColor(ren, 255, 120, 0, 70); // Stronger Orange outer

            for (int i = 0; i < 4; i++) {
                int s = (int)(baseSize * scales[i]);
                blobs[i] = (SDL_Rect){screenX + offsetsX[i] - s/2, screenY + offsetsY[i] - s/2, s, s};
                SDL_RenderFillRect(ren, &blobs[i]);
            }

            // Red Core cluster for intense storms
            if (wind > 40.0f) {
                Uint8 redAlpha = (wind > 60.0f) ? 130 : 90;
                SDL_SetRenderDrawColor(ren, 255, 20, 20, redAlpha);
                int coreSize = baseSize / 2;
                for (int i = 0; i < 3; i++) {
                    int s = (int)(coreSize * scales[i]);
                    SDL_Rect core = {screenX + offsetsX[i]/2 - s/2, screenY + offsetsY[i]/2 - s/2, s, s};
                    SDL_RenderFillRect(ren, &core);
                }
            }
          }
        }
      }
    }

    if (finalPath) {
      // Draw path with thicker, more visible lines
      SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
      // S-curve (easeInOut) easing using smoothstep: 3t^2 - 2t^3
      float eased = pathDrawProgress * pathDrawProgress * (3.0f - 2.0f * pathDrawProgress);
      int drawCount = (int)((pathLen - 1) * eased);
      for (int i = 0; i < drawCount; i++) {
        float wx1 = finalPath[i].c * GRID_SCALE - mapWidth / 2.0f,
              wy1 = finalPath[i].r * GRID_SCALE - mapHeight / 2.0f;
        float wx2 = finalPath[i + 1].c * GRID_SCALE - mapWidth / 2.0f,
              wy2 = finalPath[i + 1].r * GRID_SCALE - mapHeight / 2.0f;
        if (fabs(wx1 - wx2) < mapWidth / 2) {
          int sx1 = worldToScreenX(wx1), sy1 = worldToScreenY(wy1);
          int sx2 = worldToScreenX(wx2), sy2 = worldToScreenY(wy2);
          
          // Default color logic
          float wind = weatherGrid[finalPath[i].r * gridW + finalPath[i].c].windSpeed;
          float wave = weatherGrid[finalPath[i].r * gridW + finalPath[i].c].waveHeight;
          SDL_Color pathColor = {0, 180, 255, 255}; // Default Cyan
          if (wind > 40.0f || wave > 1.5f) {
              pathColor = (SDL_Color){255, 100, 0, 255}; // Storm Orange/Red
          }

          // Green Trail Logic
          if (showJourneyPanel && pathHoverIndex != -1) {
              if (i < pathHoverIndex) {
                  pathColor = (SDL_Color){100, 255, 150, 255}; // Bright Green
              } else if (i == pathHoverIndex) {
                  // Sub-segment green logic: Draw segment in two parts
                  int midX = sx1 + (int)((sx2 - sx1) * pathHoverT);
                  int midY = sy1 + (int)((sy2 - sy1) * pathHoverT);
                  
                  // Part 1: Green
                  SDL_Color greenTrail = {100, 255, 150, 255};
                  for (int offset = -2; offset <= 2; offset++) {
                    SDL_SetRenderDrawColor(ren, greenTrail.r, greenTrail.g, greenTrail.b, offset == 0 ? 255 : 100);
                    SDL_RenderDrawLine(ren, sx1 + offset, sy1, midX + offset, midY);
                    SDL_RenderDrawLine(ren, sx1, sy1 + offset, midX, midY + offset);
                  }
                  // Update sx1, sy1 for the remaining part
                  sx1 = midX; sy1 = midY;
              }
          }

          // Draw the (rest of) segment
          for (int offset = -2; offset <= 2; offset++) {
            SDL_SetRenderDrawColor(ren, pathColor.r, pathColor.g, pathColor.b, offset == 0 ? 255 : 100);
            SDL_RenderDrawLine(ren, sx1 + offset, sy1, sx2 + offset, sy2);
            SDL_RenderDrawLine(ren, sx1, sy1 + offset, sx2, sy2 + offset);
          }
        }
      }

      // Draw Arrow Icon at hover point
      if (showJourneyPanel && pathHoverIndex != -1 && iconArrow) {
          float wx1 = finalPath[pathHoverIndex].c * GRID_SCALE - mapWidth / 2.0f;
          float wy1 = finalPath[pathHoverIndex].r * GRID_SCALE - mapHeight / 2.0f;
          float wx2 = finalPath[pathHoverIndex+1].c * GRID_SCALE - mapWidth / 2.0f;
          float wy2 = finalPath[pathHoverIndex+1].r * GRID_SCALE - mapHeight / 2.0f;
          
          float px = wx1 + (wx2 - wx1) * pathHoverT;
          float py = wy1 + (wy2 - wy1) * pathHoverT;
          int asx = worldToScreenX(px), asy = worldToScreenY(py);
          
          float angle = atan2f(wy2 - wy1, wx2 - wx1) * 180.0f / M_PI;
          SDL_Rect arrowR = {asx - 12, asy - 12, 24, 24};
          SDL_RenderCopyEx(ren, iconArrow, NULL, &arrowR, (double)angle, NULL, SDL_FLIP_NONE);
      }
    }

    // --- Render Map Labels (Fading based on zoom) ---
    int continentAlpha = 255 - (int)((zoom - 0.4f) * (255.0f / 0.3f));
    if (continentAlpha < 0) continentAlpha = 0;
    if (continentAlpha > 255) continentAlpha = 255;

    int countryAlpha = (int)((zoom - 0.5f) * (255.0f / 0.3f));
    if (countryAlpha < 0) countryAlpha = 0;
    if (countryAlpha > 255) countryAlpha = 255;

    if (continentAlpha > 0) {
      for (int i = 0; i < continent_count; i++) {
        float px = lonToPixelX((float)continents[i].lon) - mapWidth / 2.0f;
        float py = latToPixelY((float)continents[i].lat) - mapHeight / 2.0f;
        int sx = worldToScreenX(px);
        int sy = worldToScreenY(py);

        if (sx > -200 && sx < currW + 200 && sy > -50 && sy < currH + 50) {
            if (!continents[i].tex) {
                SDL_Surface *ts = TTF_RenderText_Blended(valueFont, continents[i].name, (SDL_Color){220, 220, 220, 255});
                if (ts) {
                    continents[i].tex = SDL_CreateTextureFromSurface(ren, ts);
                    SDL_SetTextureBlendMode(continents[i].tex, SDL_BLENDMODE_BLEND);
                    continents[i].w = ts->w;
                    continents[i].h = ts->h;
                    SDL_FreeSurface(ts);
                }
            }
            if (continents[i].tex) {
                SDL_SetTextureAlphaMod(continents[i].tex, continentAlpha);
                SDL_Rect r = {sx - continents[i].w / 2, sy - continents[i].h / 2, continents[i].w, continents[i].h};
                SDL_RenderCopy(ren, continents[i].tex, NULL, &r);
            }
        }
      }
    }

    if (countryAlpha > 0) {
      for (int i = 0; i < country_count; i++) {
        float px = lonToPixelX((float)countries[i].lon) - mapWidth / 2.0f;
        float py = latToPixelY((float)countries[i].lat) - mapHeight / 2.0f;
        int sx = worldToScreenX(px);
        int sy = worldToScreenY(py);

        if (sx > -100 && sx < currW + 100 && sy > -30 && sy < currH + 30) {
            if (!countries[i].tex) {
                SDL_Surface *ts = TTF_RenderText_Blended(labelFont, countries[i].name, (SDL_Color){170, 170, 170, 255});
                if (ts) {
                    countries[i].tex = SDL_CreateTextureFromSurface(ren, ts);
                    SDL_SetTextureBlendMode(countries[i].tex, SDL_BLENDMODE_BLEND);
                    countries[i].w = ts->w;
                    countries[i].h = ts->h;
                    SDL_FreeSurface(ts);
                }
            }
            if (countries[i].tex) {
                SDL_SetTextureAlphaMod(countries[i].tex, countryAlpha);
                SDL_Rect r = {sx - countries[i].w / 2, sy - countries[i].h / 2, countries[i].w, countries[i].h};
                SDL_RenderCopy(ren, countries[i].tex, NULL, &r);
            }
        }
      }
    }

    Point *pts[2] = {&p1, &p2};
    // --- Marine Conditions Panel ---
    // Top Left Panel (Moved from Bottom Right)
    SDL_Rect marinePanel = {10, 10, 300, 128};
    drawRoundedRect(ren, marinePanel, 12, (SDL_Color){10, 30, 50, 130}, 1);
    drawRoundedRectBorder(ren, marinePanel, 12, (SDL_Color){0, 150, 255, 255});

    // Header
    SDL_Surface *marineTitle = TTF_RenderText_Blended(
        titleFont, "MARINE CONDITIONS", (SDL_Color){0, 200, 255, 255});
    SDL_Texture *marineHeaderTex =
        SDL_CreateTextureFromSurface(ren, marineTitle);
    SDL_Rect marineHeaderRect = {marinePanel.x + 15, marinePanel.y + 10,
                                 marineTitle->w, marineTitle->h};
    SDL_RenderCopy(ren, marineHeaderTex, NULL, &marineHeaderRect);
    SDL_FreeSurface(marineTitle);
    SDL_DestroyTexture(marineHeaderTex);

    // Get mouse pos for current conditions
    int mx, my;
    SDL_GetMouseState(&mx, &my);
    float mWorldX = screenToWorldX(mx);
    float mWorldY = screenToWorldY(my);
    int mc = (int)(mWorldX + mapWidth / 2.0f) / GRID_SCALE;
    int mr = (int)(mWorldY + mapHeight / 2.0f) / GRID_SCALE;

    float curWind = 0, curWave = 0, curDir = 0;
    char locText[32] = "No Data";

    if (mc >= 0 && mc < gridW && mr >= 0 && mr < gridH) {
      curWind = weatherGrid[mr * gridW + mc].windSpeed;
      curWave = weatherGrid[mr * gridW + mc].waveHeight;
      curDir = weatherGrid[mr * gridW + mc].windDir;
      snprintf(locText, sizeof(locText), "%.5f, %.5f",
               pixelToLat(mr * GRID_SCALE), pixelToLon(mc * GRID_SCALE));
    }

    int myY = marinePanel.y + 40;

    // Wind Speed
    char buf[64];
    snprintf(buf, sizeof(buf), "Wind: %.1f km/h", curWind);
    SDL_Surface *wSurf =
        TTF_RenderText_Blended(valueFont, buf, (SDL_Color){255, 255, 255, 255});
    SDL_Texture *wTex = SDL_CreateTextureFromSurface(ren, wSurf);
    SDL_Rect wRect = {marinePanel.x + 15, myY, wSurf->w, wSurf->h};
    SDL_RenderCopy(ren, wTex, NULL, &wRect);
    SDL_FreeSurface(wSurf);
    SDL_DestroyTexture(wTex);
    myY += 25;

    // Wave Height
    snprintf(buf, sizeof(buf), "Waves: %.1f m", curWave);
    SDL_Surface *hSurf =
        TTF_RenderText_Blended(valueFont, buf, (SDL_Color){100, 200, 255, 255});
    SDL_Texture *hTex = SDL_CreateTextureFromSurface(ren, hSurf);
    SDL_Rect hRect = {marinePanel.x + 15, myY, hSurf->w, hSurf->h};
    SDL_RenderCopy(ren, hTex, NULL, &hRect);
    SDL_FreeSurface(hSurf);
    SDL_DestroyTexture(hTex);
    myY += 25;

    // Direction
    snprintf(buf, sizeof(buf), "Direction: %.0f deg", curDir);
    SDL_Surface *dSurf =
        TTF_RenderText_Blended(labelFont, buf, (SDL_Color){200, 200, 200, 255});
    SDL_Texture *dTex = SDL_CreateTextureFromSurface(ren, dSurf);
    SDL_Rect dRect = {marinePanel.x + 15, myY, dSurf->w, dSurf->h};
    SDL_RenderCopy(ren, dTex, NULL, &dRect);
    SDL_FreeSurface(dSurf);
    SDL_DestroyTexture(dTex);

    // --- Storm Toggle Button (below marine panel) ---
    SDL_Color btnBg = showStorms ? (SDL_Color){10, 50, 20, 220}
                                 : (SDL_Color){50, 10, 10, 220};
    SDL_Color btnBorder = showStorms ? (SDL_Color){0, 200, 80, 255}
                                     : (SDL_Color){200, 40, 40, 255};
    drawRoundedRect(ren, stormBtn, 8, btnBg, 0);
    drawRoundedRectBorder(ren, stormBtn, 8, btnBorder);

    // Indicator dot
    SDL_Texture *tIcon = showStorms ? iconGraphOn : iconGraphOff;
    if (tIcon) {
       SDL_Rect iconR = {stormBtn.x + 8, stormBtn.y + 8, 22, 22};
       SDL_RenderCopy(ren, tIcon, NULL, &iconR);
    }

    const char *btnLabel =
        showStorms ? "Storm Graphics: ON" : "Storm Graphics: OFF";
    SDL_Color btnTextCol = showStorms ? (SDL_Color){100, 255, 130, 255}
                                      : (SDL_Color){255, 100, 100, 255};
    SDL_Surface *btnSurf =
        TTF_RenderText_Blended(labelFont, btnLabel, btnTextCol);
    SDL_Texture *btnTex = SDL_CreateTextureFromSurface(ren, btnSurf);
    SDL_Rect btnTextRect = {stormBtn.x + 40,
                            stormBtn.y + stormBtn.h / 2 - btnSurf->h / 2,
                            btnSurf->w, btnSurf->h};
    SDL_RenderCopy(ren, btnTex, NULL, &btnTextRect);
    SDL_FreeSurface(btnSurf);
    SDL_DestroyTexture(btnTex);

    // --- Coordinate info text (shifted down to avoid button overlap) ---
    const char *labels[2] = {"A", "B"};
    SDL_Texture *icons[2] = {startTex, endTex};
    for (int i = 0; i < 2; i++) {
      if (pts[i]->valid) {
        SDL_SetTextureAlphaMod(icons[i], (Uint8)pts[i]->alpha);
        SDL_Rect r = {worldToScreenX(pts[i]->x) - 12,
                      worldToScreenY(pts[i]->y) - 12, 25, 25};
        SDL_RenderCopy(ren, icons[i], NULL, &r);
        SDL_Surface *s = TTF_RenderText_Blended(smallFont, labels[i],
                                                (SDL_Color){255, 0, 0, 255});
        SDL_Texture *t = SDL_CreateTextureFromSurface(ren, s);
        SDL_Rect tr = {worldToScreenX(pts[i]->x) + 8,
                       worldToScreenY(pts[i]->y) - 20, s->w, s->h};
        SDL_RenderCopy(ren, t, NULL, &tr);
        SDL_FreeSurface(s);
        SDL_DestroyTexture(t);
      }
    }

    // Removed: if (p1.valid || p2.valid) { ... } (Yellow debug text)

    // --- AEGIS Branding Panel (Top Right) ---
    SDL_Rect aegisPanel = {currW - 320, 10, 300, 88};
    // Draw filled rounded rectangle with glass effect
    drawRoundedRect(ren, aegisPanel, 12, (SDL_Color){10, 27, 46, 195}, 1);
    drawRoundedRectBorder(ren, aegisPanel, 12, (SDL_Color){0, 180, 200, 255});

    // "AEGIS" Title
    SDL_Surface *aegisSurf = TTF_RenderText_Blended(
        aegisFont, "AEGIS", (SDL_Color){255, 255, 255, 255});
    SDL_Texture *aegisTex = SDL_CreateTextureFromSurface(ren, aegisSurf);
    // Draw 1:1 scale to avoid blur, just move it up by 20px (from 15 to -5)
    SDL_Rect aegisRect = {aegisPanel.x + 20, aegisPanel.y +4,
                          aegisSurf->w, aegisSurf->h};
    SDL_RenderCopy(ren, aegisTex, NULL, &aegisRect);
    SDL_FreeSurface(aegisSurf);
    SDL_DestroyTexture(aegisTex);

    // Subtitle
    SDL_Surface *subSurf = TTF_RenderText_Blended(
        smallFont, "Ocean Routing Intelligence", (SDL_Color){0, 180, 255, 255});
    SDL_Texture *subTex = SDL_CreateTextureFromSurface(ren, subSurf);
    SDL_Rect subRect = {aegisPanel.x + 22, aegisPanel.y + 41, subSurf->w,
                        subSurf->h};
    SDL_RenderCopy(ren, subTex, NULL, &subRect);
    SDL_FreeSurface(subSurf);
    SDL_DestroyTexture(subTex);

    // Status Icon + Text
    if (iconOnline && iconUnplug) {
       SDL_Texture *statusTex = isOnline ? iconOnline : iconUnplug;
       // Original Y was 57, now 62. Original size 16, now 21.
       SDL_Rect onR = {aegisPanel.x + 20, aegisPanel.y + 62, 21, 21};
       SDL_RenderCopy(ren, statusTex, NULL, &onR);
    }
    // Toggle text and color based on network status
    const char* netStatusText = isOnline ? "SYSTEM ONLINE" : "SYSTEM OFFLINE";
    SDL_Color netStatusColor = isOnline ? (SDL_Color){100, 255, 150, 255} : (SDL_Color){255, 50, 50, 255};
    drawGlowingText(ren, statusFont, netStatusText, netStatusColor, aegisPanel.x + 45, aegisPanel.y + 64);

    // Simulation Label (if active)
    if (simulationMode) {
      SDL_Surface *simSurf = TTF_RenderText_Blended(
          smallFont, "[SIMULATION MODE]", (SDL_Color){255, 100, 100, 255});
      SDL_Texture *simTex = SDL_CreateTextureFromSurface(ren, simSurf);
      SDL_Rect simRect = {aegisPanel.x + 20, aegisPanel.y + 85, simSurf->w,
                          simSurf->h};
      SDL_RenderCopy(ren, simTex, NULL, &simRect);
      SDL_FreeSurface(simSurf);
      SDL_DestroyTexture(simTex);
    }

    // --- Smart Routing Progress Dialogue ---
    if (smartRouteAnim > 0.01f) {
        SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
        Uint8 alpha = (Uint8)(255 * smartRouteAnim);
        
        int boxW = 380, boxH = 140;
        SDL_Rect boxRect = {currW/2 - boxW/2, currH/2 - boxH/2, boxW, boxH};
        
        // Modal Shadow/Background
        drawRoundedRect(ren, boxRect, 10, (SDL_Color){15, 20, 30, (Uint8)(alpha * 0.9f)}, 1);
        drawRoundedRectBorder(ren, boxRect, 10, (SDL_Color){0, 255, 255, alpha});
        
        // Header
        SDL_Surface *sHead = TTF_RenderText_Blended(titleFont, "SMART ROUTING VERIFICATION", (SDL_Color){255, 255, 255, alpha});
        SDL_Texture *tHead = SDL_CreateTextureFromSurface(ren, sHead);
        SDL_Rect rHead = {boxRect.x + boxW/2 - sHead->w/2, boxRect.y + 20, sHead->w, sHead->h};
        SDL_RenderCopy(ren, tHead, NULL, &rHead);
        SDL_FreeSurface(sHead);
        SDL_DestroyTexture(tHead);
        
        // Status Label
        const char *stLabel = "Status: Verifying Path...";
        if (smartRouteStage == 2) stLabel = "Status: API Calling...";
        if (smartRouteStage == 3) stLabel = "Status: Optimizing Path...";
        SDL_Surface *sStat = TTF_RenderText_Blended(labelFont, stLabel, (SDL_Color){0, 180, 255, alpha});
        SDL_Texture *tStat = SDL_CreateTextureFromSurface(ren, sStat);
        SDL_Rect rStat = {boxRect.x + boxW/2 - sStat->w/2 + 20, boxRect.y + 75, sStat->w, sStat->h};
        SDL_RenderCopy(ren, tStat, NULL, &rStat);
        SDL_FreeSurface(sStat);
        SDL_DestroyTexture(tStat);
        
        // Spinner (Icon instead of lines)
        if (iconLoader) {
            SDL_Rect lRect = {boxRect.x + boxW/2 - sStat->w/2 - 25, boxRect.y + 73, 20, 20};
            SDL_Point center = {10, 10};
            SDL_RenderCopyEx(ren, iconLoader, NULL, &lRect, loadingRotation * 2.0f, &center, SDL_FLIP_NONE);
        }
    }

    // --- Route Analysis Panel ---
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    SDL_Rect panel = {currW - 320, 210, 300, 280}; // Moved down 200px total

    // Draw rounded panel (no glossy effect)
    // Draw rounded panel with glass effect
    drawRoundedRect(ren, panel, 12, (SDL_Color){10, 27, 46, 130}, 1);

    // Draw rounded border stroke
    drawRoundedRectBorder(ren, panel, 12, (SDL_Color){0, 180, 200, 255});

    int yPos = panel.y + 15;

    // Title: "ROUTE ANALYSIS" with cyan indicator dot
    SDL_SetRenderDrawColor(ren, 0, 255, 255, 255);
    SDL_Rect dot = {panel.x + panel.w - 20, yPos + 5, 8, 8};
    SDL_RenderFillRect(ren, &dot);

    drawGlowingText(ren, titleFont, "ROUTE ANALYSIS", (SDL_Color){0, 255, 255, 255}, panel.x + 15, yPos);
    
    // Loading Spinner
    if (spinnerTimer > 0) {
        SDL_SetRenderDrawColor(ren, 0, 255, 255, 200);
        int spinnerX = panel.x + 180;
        int spinnerY = yPos + 10;
        for (int i=0; i<8; i++) {
            float ang = (loadingRotation + i * 45) * M_PI / 180.0f;
            int x1 = spinnerX + cosf(ang) * 4;
            int y1 = spinnerY + sinf(ang) * 4;
            int x2 = spinnerX + cosf(ang) * 8;
            int y2 = spinnerY + sinf(ang) * 8;
            SDL_RenderDrawLine(ren, x1, y1, x2, y2);
        }
    }
    yPos += 35;

    // Status
    SDL_Surface *statusLabel = TTF_RenderText_Blended(
        labelFont, "Status", (SDL_Color){150, 150, 150, 255});
    SDL_Texture *statusLabelTex =
        SDL_CreateTextureFromSurface(ren, statusLabel);
    SDL_Rect statusLabelRect = {panel.x + 15, yPos, statusLabel->w,
                                statusLabel->h};
    SDL_RenderCopy(ren, statusLabelTex, NULL, &statusLabelRect);
    SDL_FreeSurface(statusLabel);
    SDL_DestroyTexture(statusLabelTex);

    SDL_Color statusColor = strcmp(routeStatus, "OPTIMIZED") == 0
                                ? (SDL_Color){0, 255, 100, 255}
                                : (SDL_Color){255, 200, 0, 255};
    SDL_Surface *statusVal =
        TTF_RenderText_Blended(valueFont, routeStatus, statusColor);
    SDL_Texture *statusValTex = SDL_CreateTextureFromSurface(ren, statusVal);
    SDL_Rect statusValRect = {panel.x + panel.w - 15 - statusVal->w, yPos - 2,
                              statusVal->w, statusVal->h};
    SDL_RenderCopy(ren, statusValTex, NULL, &statusValRect);
    SDL_FreeSurface(statusVal);
    SDL_DestroyTexture(statusValTex);
    yPos += 35;

    // Total Distance
    SDL_Surface *distLabel = TTF_RenderText_Blended(
        labelFont, "Total Distance", (SDL_Color){150, 150, 150, 255});
    SDL_Texture *distLabelTex = SDL_CreateTextureFromSurface(ren, distLabel);
    SDL_Rect distLabelRect = {panel.x + 15, yPos, distLabel->w, distLabel->h};
    SDL_RenderCopy(ren, distLabelTex, NULL, &distLabelRect);
    SDL_FreeSurface(distLabel);
    SDL_DestroyTexture(distLabelTex);

    char distStr[64];
    snprintf(distStr, sizeof(distStr), "%.0f", routeDistance);
    SDL_Surface *distVal = TTF_RenderText_Blended(
        valueFont, distStr, (SDL_Color){255, 255, 255, 255});
    SDL_Texture *distValTex = SDL_CreateTextureFromSurface(ren, distVal);
    SDL_Rect distValRect = {panel.x + panel.w - 70 - distVal->w, yPos - 2,
                            distVal->w, distVal->h};
    SDL_RenderCopy(ren, distValTex, NULL, &distValRect);
    SDL_FreeSurface(distVal);
    SDL_DestroyTexture(distValTex);

    SDL_Surface *kmLabel = TTF_RenderText_Blended(
        labelFont, "km", (SDL_Color){100, 100, 100, 255});
    SDL_Texture *kmLabelTex = SDL_CreateTextureFromSurface(ren, kmLabel);
    SDL_Rect kmLabelRect = {panel.x + panel.w - 40, yPos, kmLabel->w,
                            kmLabel->h};
    SDL_RenderCopy(ren, kmLabelTex, NULL, &kmLabelRect);
    SDL_FreeSurface(kmLabel);
    SDL_DestroyTexture(kmLabelTex);
    yPos += 35;

    // ETA
    SDL_Surface *etaLabel = TTF_RenderText_Blended(
        labelFont, "ETA", (SDL_Color){150, 150, 150, 255});
    SDL_Texture *etaLabelTex = SDL_CreateTextureFromSurface(ren, etaLabel);
    SDL_Rect etaLabelRect = {panel.x + 15, yPos, etaLabel->w, etaLabel->h};
    SDL_RenderCopy(ren, etaLabelTex, NULL, &etaLabelRect);
    SDL_FreeSurface(etaLabel);
    SDL_DestroyTexture(etaLabelTex);

    char etaStr[64];
    snprintf(etaStr, sizeof(etaStr), "%.1f", routeETA);
    int etaW, etaH;
    TTF_SizeText(valueFont, etaStr, &etaW, &etaH);
    drawGlowingText(ren, valueFont, etaStr, (SDL_Color){0, 255, 255, 255}, panel.x + panel.w - 80 - etaW, yPos - 2);

    SDL_Surface *daysLabel = TTF_RenderText_Blended(
        labelFont, "days", (SDL_Color){100, 100, 100, 255});
    SDL_Texture *daysLabelTex = SDL_CreateTextureFromSurface(ren, daysLabel);
    SDL_Rect daysLabelRect = {panel.x + panel.w - 55, yPos, daysLabel->w,
                              daysLabel->h};
    SDL_RenderCopy(ren, daysLabelTex, NULL, &daysLabelRect);
    SDL_FreeSurface(daysLabel);
    SDL_DestroyTexture(daysLabelTex);
    yPos += 40;

    // Divider line
    SDL_SetRenderDrawColor(ren, 40, 60, 80, 255);
    SDL_RenderDrawLine(ren, panel.x + 15, yPos, panel.x + panel.w - 15, yPos);
    yPos += 15;

    // Light background for stats (using dark theme now)
    SDL_Rect statsBg = {panel.x + 10, yPos - 5, panel.w - 20, 65};
    drawRoundedRect(ren, statsBg, 8, (SDL_Color){10, 27, 46, 195}, 0);

    // Fuel Estimate
    SDL_Surface *fuelLabel = TTF_RenderText_Blended(
        labelFont, "Fuel Estimate", (SDL_Color){0, 180, 200, 255});
    SDL_Texture *fuelLabelTex = SDL_CreateTextureFromSurface(ren, fuelLabel);
    SDL_Rect fuelLabelRect = {panel.x + 20, yPos, fuelLabel->w, fuelLabel->h};
    SDL_RenderCopy(ren, fuelLabelTex, NULL, &fuelLabelRect);
    SDL_FreeSurface(fuelLabel);
    SDL_DestroyTexture(fuelLabelTex);

    char fuelStr[64];
    snprintf(fuelStr, sizeof(fuelStr), "%.1f tons", routeFuel);
    SDL_Surface *fuelVal = TTF_RenderText_Blended(
        valueFont, fuelStr, (SDL_Color){200, 200, 200, 255});
    SDL_Texture *fuelValTex = SDL_CreateTextureFromSurface(ren, fuelVal);
    SDL_Rect fuelValRect = {panel.x + panel.w - 15 - fuelVal->w, yPos,
                            fuelVal->w, fuelVal->h};
    SDL_RenderCopy(ren, fuelValTex, NULL, &fuelValRect);
    SDL_FreeSurface(fuelVal);
    SDL_DestroyTexture(fuelValTex);
    yPos += 25;

    // Max Risk
    SDL_Surface *riskLabel = TTF_RenderText_Blended(
        labelFont, "Max Risk", (SDL_Color){0, 180, 200, 255});
    SDL_Texture *riskLabelTex = SDL_CreateTextureFromSurface(ren, riskLabel);
    SDL_Rect riskLabelRect = {panel.x + 20, yPos, riskLabel->w, riskLabel->h};
    SDL_RenderCopy(ren, riskLabelTex, NULL, &riskLabelRect);
    SDL_FreeSurface(riskLabel);
    SDL_DestroyTexture(riskLabelTex);

    char riskStr[64];
    snprintf(riskStr, sizeof(riskStr), "%.0f%%", routeRisk);
    SDL_Surface *riskVal = TTF_RenderText_Blended(
        valueFont, riskStr, (SDL_Color){0, 255, 100, 255});
    SDL_Texture *riskValTex = SDL_CreateTextureFromSurface(ren, riskVal);
    SDL_Rect riskValRect = {panel.x + panel.w - 15 - riskVal->w, yPos,
                            riskVal->w, riskVal->h};
    SDL_RenderCopy(ren, riskValTex, NULL, &riskValRect);
    SDL_FreeSurface(riskVal);
    SDL_DestroyTexture(riskValTex);
    yPos += 30;

    // Optimization note
    if (strcmp(routeStatus, "OPTIMIZED") == 0) {
      char noteStr[128];
      snprintf(noteStr, sizeof(noteStr), "*V5 Optimized: %.0fkm*",
               routeDistance);
      SDL_Surface *note = TTF_RenderText_Blended(
          labelFont, noteStr, (SDL_Color){100, 120, 140, 255});
      SDL_Texture *noteTex = SDL_CreateTextureFromSurface(ren, note);
      SDL_Rect noteRect = {panel.x + 15, yPos, note->w, note->h};
      SDL_RenderCopy(ren, noteTex, NULL, &noteRect);
      SDL_FreeSurface(note);
      SDL_DestroyTexture(noteTex);
    }

    // --- Mouse Coordinate Panel (Bottom Right) ---
    // --- Coordinate Panel (A / B) ---

    char coordA[128] = "A: --";
    char coordB[128] = "B: --";

    if (p1.valid) {
      float px = worldToPixelX(p1.x);
      float py = worldToPixelY(p1.y);
      float lat = pixelToLat(py);
      float lon = pixelToLon(px);
      char tmp[96];
      formatCoord(tmp, sizeof(tmp), lat, lon);
      snprintf(coordA, sizeof(coordA), "A: %s", tmp);
    }

    if (p2.valid) {
      float px = worldToPixelX(p2.x);
      float py = worldToPixelY(p2.y);
      float lat = pixelToLat(py);
      float lon = pixelToLon(px);
      char tmp[96];
      formatCoord(tmp, sizeof(tmp), lat, lon);
      snprintf(coordB, sizeof(coordB), "B: %s", tmp);
    }

    int aw, ah, bw, bh;
    TTF_SizeText(valueFont, coordA, &aw, &ah);
    TTF_SizeText(valueFont, coordB, &bw, &bh);

    int panelW = (aw > bw ? aw : bw) + 30;
    int panelH = ah + bh + 30;

    int panelX = currW - panelW - 20;
    int panelY = currH - panelH - 20;

    SDL_Rect coordPanel = {panelX, panelY, panelW, panelH};
    drawRoundedRect(ren, coordPanel, 8, (SDL_Color){10, 27, 46, 130}, 1);
    drawRoundedRectBorder(ren, coordPanel, 8, (SDL_Color){0, 180, 200, 255});

    drawGlowingText(ren, valueFont, coordA, (SDL_Color){0, 255, 255, 255}, panelX + 15, panelY + 10);
    drawGlowingText(ren, valueFont, coordB, (SDL_Color){0, 255, 255, 255}, panelX + 15, panelY + 15 + ah);

    // vessel info panel
    // --- Vessel Info Panel (Table Style) ---
    int vesselPanelW = 300; // same width as ROUTE ANALYSIS panel
    int vesselPanelH = 90;
    int vesselPanelX = currW - vesselPanelW - 20; // right aligned
    int vesselPanelY = 108;                       // below top

    SDL_Rect vesselPanel = {vesselPanelX, vesselPanelY, vesselPanelW,
                            vesselPanelH};

    drawRoundedRect(ren, vesselPanel, 12, (SDL_Color){10, 27, 46, 130}, 1);
    drawRoundedRectBorder(ren, vesselPanel, 12, (SDL_Color){0, 180, 200, 255});

    // Column X positions
    int colVessel = vesselPanel.x + 15;
    int colSpeed = vesselPanel.x + 120;
    int colMode = vesselPanel.x + 200;

    // Header row
    SDL_Surface *hVessel = TTF_RenderText_Blended(
        labelFont, "VESSEL", (SDL_Color){120, 180, 200, 255});
    SDL_Surface *hSpeed = TTF_RenderText_Blended(
        labelFont, "SPEED", (SDL_Color){120, 180, 200, 255});
    SDL_Surface *hMode = TTF_RenderText_Blended(
        labelFont, "MODE", (SDL_Color){120, 180, 200, 255});

    SDL_Texture *hVesselT = SDL_CreateTextureFromSurface(ren, hVessel);
    SDL_Texture *hSpeedT = SDL_CreateTextureFromSurface(ren, hSpeed);
    SDL_Texture *hModeT = SDL_CreateTextureFromSurface(ren, hMode);

    SDL_RenderCopy(
        ren, hVesselT, NULL,
        &(SDL_Rect){colVessel, vesselPanel.y + 10, hVessel->w, hVessel->h});
    SDL_RenderCopy(
        ren, hSpeedT, NULL,
        &(SDL_Rect){colSpeed, vesselPanel.y + 10, hSpeed->w, hSpeed->h});
    SDL_RenderCopy(
        ren, hModeT, NULL,
        &(SDL_Rect){colMode, vesselPanel.y + 10, hMode->w, hMode->h});

    SDL_FreeSurface(hVessel);
    SDL_FreeSurface(hSpeed);
    SDL_FreeSurface(hMode);
    SDL_DestroyTexture(hVesselT);
    SDL_DestroyTexture(hSpeedT);
    SDL_DestroyTexture(hModeT);

    // Controls Help
    SDL_Surface *helpSurf = TTF_RenderText_Blended(
        smallFont,
        "Drag: Left/Right Click | Move: WASD | Zoom: Scroll | Reset: F",
        (SDL_Color){100, 150, 200, 200});
    SDL_Texture *helpTex = SDL_CreateTextureFromSurface(ren, helpSurf);
    SDL_Rect helpRect = {10, currH - 25, helpSurf->w, helpSurf->h};
    SDL_RenderCopy(ren, helpTex, NULL, &helpRect);
    SDL_FreeSurface(helpSurf);
    SDL_DestroyTexture(helpTex);

    // Divider line
    SDL_SetRenderDrawColor(ren, 40, 70, 90, 255);
    SDL_RenderDrawLine(ren, vesselPanel.x + 15, vesselPanel.y + 38,
                       vesselPanel.x + vesselPanel.w - 15, vesselPanel.y + 38);

    // Value row
    SDL_Surface *vName = TTF_RenderText_Blended(
        smallFont, shipName, (SDL_Color){255, 255, 255, 255});
    SDL_Surface *vMode = TTF_RenderText_Blended(smallFont, shipMode,
                                                (SDL_Color){0, 255, 100, 255});

    SDL_Texture *vNameT = SDL_CreateTextureFromSurface(ren, vName);
    SDL_Texture *vModeT = SDL_CreateTextureFromSurface(ren, vMode);

    SDL_RenderCopy(
        ren, vNameT, NULL,
        &(SDL_Rect){colVessel, vesselPanel.y + 45, vName->w, vName->h});
    drawGlowingText(ren, smallFont, shipSpeed, (SDL_Color){0, 255, 255, 255}, colSpeed, vesselPanel.y + 45);
    SDL_RenderCopy(
        ren, vModeT, NULL,
        &(SDL_Rect){colMode, vesselPanel.y + 45, vMode->w, vMode->h});

    SDL_FreeSurface(vName);
    SDL_FreeSurface(vMode);
    SDL_DestroyTexture(vNameT);
    SDL_DestroyTexture(vModeT);

    // --- System Configuration Button (Top Center) ---
    int configBtnW = 220;
    int configBtnH = 30;
    int configBtnX = currW / 2 - configBtnW / 2;
    int configBtnY = 15;
    SDL_Rect configBtnRect = {configBtnX, configBtnY, configBtnW, configBtnH};
    
    // Draw rounded shape
    drawRoundedRect(ren, configBtnRect, 15, (SDL_Color){15, 30, 45, 200}, 0);
    drawRoundedRectBorder(ren, configBtnRect, 15, (SDL_Color){0, 150, 200, 200});
    
    if (iconSettings) {
       SDL_Rect setR = {configBtnX + 11, configBtnY + configBtnH/2 - 9, 18, 18};
       SDL_RenderCopy(ren, iconSettings, NULL, &setR);
    }
    
    SDL_Surface *configBtnSurf = TTF_RenderText_Blended(
        labelFont, "SYSTEM CONFIGURATION", (SDL_Color){100, 200, 255, 255});
    SDL_Texture *configBtnTex = SDL_CreateTextureFromSurface(ren, configBtnSurf);
    SDL_Rect configBtnTRect = {configBtnX + 31, configBtnY + configBtnH/2 - configBtnSurf->h/2, configBtnSurf->w, configBtnSurf->h};
    SDL_RenderCopy(ren, configBtnTex, NULL, &configBtnTRect);
    SDL_FreeSurface(configBtnSurf);
    SDL_DestroyTexture(configBtnTex);


    // --- System Configuration Modal ---
    if (showConfigWindow || configAnimProgress > 0.01f) {
      SDL_Texture *prevTarget = SDL_GetRenderTarget(ren);
      SDL_Texture *modalTex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, currW, currH);
      SDL_SetTextureBlendMode(modalTex, SDL_BLENDMODE_BLEND);
      SDL_SetRenderTarget(ren, modalTex);
      SDL_SetRenderDrawColor(ren, 0, 0, 0, 0);
      SDL_RenderClear(ren);

      // Darken background
      SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
      SDL_SetRenderDrawColor(ren, 0, 0, 0, 180);
      SDL_Rect fullScreen = {0, 0, currW, currH};
      SDL_RenderFillRect(ren, &fullScreen);

      int modalW = 800, modalH = 500;
      int modalX = currW / 2 - modalW / 2;
      int modalY = currH / 2 - modalH / 2;
      SDL_Rect modalRect = {modalX, modalY, modalW, modalH};

      // Modal Base
      drawRoundedRect(ren, modalRect, 8, (SDL_Color){15, 18, 22, 250}, 0);
      drawRoundedRectBorder(ren, modalRect, 8, (SDL_Color){40, 50, 60, 255});

      // Header Area
      SDL_Rect headerRect = {modalX, modalY, modalW, 60};
      SDL_Color hdrC = {15, 18, 22, 255};
      drawRoundedRect(ren, headerRect, 8, hdrC, 0); // Need to clip bottom half manually or overdraw
      SDL_Rect hdrFill = {modalX+1, modalY+20, modalW-2, 40};
      SDL_RenderFillRect(ren, &hdrFill);
      // Header dividing line
      SDL_SetRenderDrawColor(ren, 40, 50, 60, 255);
      SDL_RenderDrawLine(ren, modalX, modalY+60, modalX+modalW, modalY+60);

      // Aegis Configuration Title
      SDL_Surface *cfgTitleS = TTF_RenderText_Blended(titleFont, "AEGIS CONFIGURATION", (SDL_Color){255, 255, 255, 255});
      SDL_Texture *cfgTitleT = SDL_CreateTextureFromSurface(ren, cfgTitleS);
      SDL_Rect cfgTitleR = {modalX + 45, modalY + 20, cfgTitleS->w, cfgTitleS->h};
      SDL_RenderCopy(ren, cfgTitleT, NULL, &cfgTitleR);
      SDL_FreeSurface(cfgTitleS);
      SDL_DestroyTexture(cfgTitleT);

      // Close 'X' Button
      SDL_SetRenderDrawColor(ren, 150, 150, 150, 255);
      SDL_RenderDrawLine(ren, modalX + modalW - 25, modalY + 25, modalX + modalW - 15, modalY + 35);
      SDL_RenderDrawLine(ren, modalX + modalW - 25, modalY + 35, modalX + modalW - 15, modalY + 25);

      // Left Sidebar
      SDL_Rect sidebarR = {modalX, modalY+61, 220, modalH-61};
      SDL_SetRenderDrawColor(ren, 25, 28, 35, 255);
      SDL_RenderFillRect(ren, &sidebarR);
            const char *tabs[] = {"Vessel Parameters", "Environmental API", "Weather Settings", "Keyboard & Controls"};
       SDL_Texture *tabIcons[] = {iconShip, iconApi, iconCloud, iconKeyboard};
       for (int i=0; i<4; i++) {
          int ty = modalY + 90 + (i * 50);
          if (i == configActiveTab) {
             SDL_SetRenderDrawColor(ren, 35, 40, 48, 255);
             SDL_Rect tabBg = {modalX, ty - 10, 220, 40};
             SDL_RenderFillRect(ren, &tabBg);
             // Cyan left indicator
             SDL_SetRenderDrawColor(ren, 0, 255, 255, 255);
             SDL_Rect tick = {modalX, ty - 10, 3, 40};
             SDL_RenderFillRect(ren, &tick);
          }
          SDL_Color tCol = (i == configActiveTab) ? (SDL_Color){255,255,255,255} : (SDL_Color){150,150,150,255};
          
          // Draw Tab Icon
          if (tabIcons[i]) {
             SDL_SetTextureColorMod(tabIcons[i], tCol.r, tCol.g, tCol.b);
             SDL_Rect iconR = {modalX + 15, ty, 20, 20};
             SDL_RenderCopy(ren, tabIcons[i], NULL, &iconR);
          }

          SDL_Surface *ts = TTF_RenderText_Blended(labelFont, tabs[i], tCol);
          SDL_Texture *tt = SDL_CreateTextureFromSurface(ren, ts);
          SDL_Rect tr = {modalX + 45, ty, ts->w, ts->h};
          SDL_RenderCopy(ren, tt, NULL, &tr);
          SDL_FreeSurface(ts);
          SDL_DestroyTexture(tt);
       }

      // Main Content Area
      int cx = modalX + 250;
      int cy = modalY + 90;

      if (configActiveTab == 0) {
         // Vessel Parameters
         SDL_Surface *htTitle = TTF_RenderText_Blended(titleFont, "Vessel Parameters", (SDL_Color){255, 255, 255, 255});
         SDL_Texture *htTitleT = SDL_CreateTextureFromSurface(ren, htTitle);
         SDL_Rect htTitleR = {cx, cy, htTitle->w, htTitle->h};
         SDL_RenderCopy(ren, htTitleT, NULL, &htTitleR);
         SDL_FreeSurface(htTitle);
         SDL_DestroyTexture(htTitleT);
         cy += 40;

         // Field Name helper
         drawBox(ren, labelFont, font, smallFont, "Vessel Name", inputShipName, NULL, cx, cy, 500, activeInputIndex == 0); cy += 80;
         drawBox(ren, labelFont, font, smallFont, "Max Speed (kts)", inputMaxSpeed, NULL, cx, cy, 230, activeInputIndex == 1);
         drawBox(ren, labelFont, font, smallFont, "Cruise Speed (kts)", inputCruiseSpeed, NULL, cx + 270, cy, 230, activeInputIndex == 2); cy += 80;
         drawBox(ren, labelFont, font, smallFont, "Fuel Consumption Rate", inputFuelRate, "tons/day", cx, cy, 230, activeInputIndex == 3);
         drawBox(ren, labelFont, font, smallFont, "Max Wave Tolerance", inputWaveTol, "meters", cx + 270, cy, 230, activeInputIndex == 4);

      } else if (configActiveTab == 1) {
          // Environmental API (Placeholder or limited)
          SDL_Surface *htTitle = TTF_RenderText_Blended(titleFont, "Environmental API Settings", (SDL_Color){255, 255, 255, 255});
          SDL_Texture *htTitleT = SDL_CreateTextureFromSurface(ren, htTitle);
          SDL_Rect htTitleR = {cx, cy, htTitle->w, htTitle->h};
          SDL_RenderCopy(ren, htTitleT, NULL, &htTitleR);
          SDL_FreeSurface(htTitle);
          SDL_DestroyTexture(htTitleT);
          cy += 60;
          drawBox(ren, labelFont, font, smallFont, "API Endpoint", "https://api.open-meteo.com/v1/forecast", NULL, cx, cy, 500, 0);
          cy += 100;
          // Clear Cache Button
          SDL_Rect cBtn = {cx, cy, 260, 45};
          SDL_SetRenderDrawColor(ren, 60, 20, 20, 180); SDL_RenderFillRect(ren, &cBtn);
          SDL_SetRenderDrawColor(ren, 255, 60, 60, 255); SDL_RenderDrawRect(ren, &cBtn);
          drawGlowingText(ren, labelFont, "CLEAR WEATHER CACHE", (SDL_Color){255, 100, 100, 255}, cBtn.x + 40, cBtn.y + 12);
          SDL_Surface *ts = TTF_RenderText_Blended(smallFont, "Resets all local path and ocean weather data", (SDL_Color){150, 150, 150, 255});
          if (ts) {
              SDL_Texture *tt = SDL_CreateTextureFromSurface(ren, ts);
              SDL_Rect tr = {cx, cy + 50, ts->w, ts->h}; SDL_RenderCopy(ren, tt, NULL, &tr);
              SDL_FreeSurface(ts); SDL_DestroyTexture(tt);
          }

      } else if (configActiveTab == 2) {
         // Weather Settings
         SDL_Surface *htTitle = TTF_RenderText_Blended(titleFont, "Weather Visualizations", (SDL_Color){255, 255, 255, 255});
         SDL_Texture *htTitleT = SDL_CreateTextureFromSurface(ren, htTitle);
         SDL_Rect htTitleR = {cx, cy, htTitle->w, htTitle->h};
         SDL_RenderCopy(ren, htTitleT, NULL, &htTitleR);
         SDL_FreeSurface(htTitle);
         SDL_DestroyTexture(htTitleT);
         cy += 50;
         
         // Inner Group Card
         SDL_Rect groupR = {cx - 15, cy - 10, 530, 240};
         drawRoundedRect(ren, groupR, 8, (SDL_Color){20, 24, 30, 180}, 0);
         drawRoundedRectBorder(ren, groupR, 8, (SDL_Color){40, 50, 60, 255});
         
         drawToggle(ren, labelFont, smallFont, "Master Weather Layer", "Toggle all map weather overlays", cx, cy, 510, showStorms);
         // Dividers
         SDL_SetRenderDrawColor(ren, 40, 50, 60, 255);
         SDL_RenderDrawLine(ren, cx + 50, cy + 60, cx + 510, cy + 60);
         cy += 80;
         
         // Sub-items indent (Visual structure)
         SDL_SetRenderDrawColor(ren, 40, 50, 60, 255);
         SDL_RenderDrawLine(ren, cx + 30, cy - 15, cx + 30, cy + 45); // Left vertical bar
         
         drawToggle(ren, labelFont, smallFont, "Show Wind Particles", "Animated particle flow for wind direction/speed", cx + 50, cy, 460, showWindParticles);
         SDL_RenderDrawLine(ren, cx + 80, cy + 60, cx + 510, cy + 60);
         cy += 80;
         
         drawToggle(ren, labelFont, smallFont, "Show Wave Heights", "Color gradient overlay for ocean swells", cx + 50, cy, 460, showWaveHeights);

      } else if (configActiveTab == 3) {
         // Keyboard & Controls
         SDL_Surface *htTitle = TTF_RenderText_Blended(titleFont, "Keyboard & Map Controls", (SDL_Color){255, 255, 255, 255});
         SDL_Texture *htTitleT = SDL_CreateTextureFromSurface(ren, htTitle);
         SDL_Rect htTitleR = {cx, cy, htTitle->w, htTitle->h};
         SDL_RenderCopy(ren, htTitleT, NULL, &htTitleR);
         SDL_FreeSurface(htTitle);
         SDL_DestroyTexture(htTitleT);
         cy += 50;

         // Map Navigation Block
         drawBlock(ren, smallFont, "MAP NAVIGATION", cx, cy, 230, 180);
         drawKeyRow(ren, labelFont, smallFont, "Pan Map", "Left Click", "+ Drag", NULL, cx, cy + 40, 230);
         drawKeyRow(ren, labelFont, smallFont, "Zoom", "Scroll", NULL, NULL, cx, cy + 70, 230);

         // Routing Points Block
         drawBlock(ren, smallFont, "ROUTING POINTS", cx + 250, cy, 250, 180);
         drawKeyRow(ren, labelFont, smallFont, "Set Start Point", "Click 1", NULL, "Places cyan start marker", cx + 250, cy + 40, 250);
         drawKeyRow(ren, labelFont, smallFont, "Set End Point", "Click 2", NULL, "Places red target marker", cx + 250, cy + 90, 250);
         drawKeyRow(ren, labelFont, smallFont, "Reset Route", "Click 3", NULL, "Clears current path", cx + 250, cy + 140, 250);
      }

      // Bottom Area Buttons
      // Separator
      SDL_SetRenderDrawColor(ren, 40, 50, 60, 255);
      SDL_RenderDrawLine(ren, modalX + 220, modalY + modalH - 70, modalX + modalW, modalY + modalH - 70);

      // Cancel
      SDL_Surface *btnCancel = TTF_RenderText_Blended(labelFont, "Cancel", (SDL_Color){150, 150, 150, 255});
      SDL_Texture *texCancel = SDL_CreateTextureFromSurface(ren, btnCancel);
      SDL_Rect rCancel = {modalX + modalW - 220, modalY + modalH - 45, btnCancel->w, btnCancel->h};
      SDL_RenderCopy(ren, texCancel, NULL, &rCancel);
      SDL_FreeSurface(btnCancel);
      SDL_DestroyTexture(texCancel);

      // Apply
      SDL_Rect RApply = {modalX + modalW - 160, modalY + modalH - 55, 140, 35};
      drawRoundedRect(ren, RApply, 4, (SDL_Color){0, 255, 230, 255}, 0);
      SDL_Surface *btnApply = TTF_RenderText_Blended(labelFont, "Apply Configuration", (SDL_Color){0, 0, 0, 255});
      SDL_Texture *texApply = SDL_CreateTextureFromSurface(ren, btnApply);
      SDL_Rect rApply = {RApply.x + RApply.w/2 - btnApply->w/2, RApply.y + RApply.h/2 - btnApply->h/2, btnApply->w, btnApply->h};
      SDL_RenderCopy(ren, texApply, NULL, &rApply);
      SDL_FreeSurface(btnApply);
      SDL_DestroyTexture(texApply);

      SDL_SetRenderTarget(ren, prevTarget);
      SDL_SetTextureBlendMode(modalTex, SDL_BLENDMODE_BLEND);
      SDL_SetTextureAlphaMod(modalTex, (Uint8)(255 * configAnimProgress));
      SDL_Rect renderScreen = {0, 0, currW, currH};
      SDL_RenderCopy(ren, modalTex, NULL, &renderScreen);
      SDL_DestroyTexture(modalTex);
    }

    if (isHoveringPath && !showConfigWindow && !showJourneyPanel) {
        char ttText[128];
        snprintf(ttText, sizeof(ttText), "Wind: %.1f km/h  Wave: %.1f m  Dir: %.0f", hoverWind, (hoverWave < 0 ? 0 : hoverWave), hoverDir);
        SDL_Surface *ts = TTF_RenderText_Blended(font, ttText, (SDL_Color){255, 255, 255, 255});
        SDL_Texture *tt = SDL_CreateTextureFromSurface(ren, ts);
        int tw = ts->w, th = ts->h;
        int tx = mouseX + 15, ty = mouseY + 15;
        if (tx + tw + 10 > winWidth) tx = mouseX - tw - 15;
        if (ty + th + 10 > winHeight) ty = mouseY - th - 15;
        
        SDL_Rect bg = {tx - 5, ty - 5, tw + 10, th + 10};
        SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
        SDL_SetRenderDrawColor(ren, 10, 20, 30, 230);
        SDL_RenderFillRect(ren, &bg);
        SDL_SetRenderDrawColor(ren, 0, 255, 255, 255);
        SDL_RenderDrawRect(ren, &bg);
        
        SDL_Rect tr = {tx, ty, tw, th};
        SDL_RenderCopy(ren, tt, NULL, &tr);
        SDL_FreeSurface(ts);
        SDL_DestroyTexture(tt);
    }

    // --- Journey Planning UI ---
    drawPlanJourneyButton(ren, titleFont);
    drawJourneySidebar(ren, aegisFont, labelFont, valueFont);

    SDL_RenderPresent(ren);
  }
  TTF_CloseFont(font);
  TTF_CloseFont(smallFont);
  TTF_CloseFont(valueFont);
  TTF_CloseFont(labelFont);
  TTF_CloseFont(titleFont);
  TTF_CloseFont(aegisFont);
  SDL_Quit();
  return 0;
}
