// https://github.com/darkskyapp/skycons/blob/master/skycons.js
#include "application.h"
#include "math.h"
// #include "skycons.h"

#define W                   48
#define H                   48

#define W_OFFSET            0
#define H_OFFSET            16

#define PI                  3.14159265358979323846
#define TAU                 2.0 * PI
#define TWO_OVER_SQRT_2     2.0 / sqrt(2)

#define S                   min(W, H)

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
Adafruit_SSD1306 display(D0);


static float WIND_PATHS[2][128] = {{
    -0.7500, -0.1800, -0.7219, -0.1527, -0.6971, -0.1225,
    -0.6739, -0.0910, -0.6516, -0.0588, -0.6298, -0.0262,
    -0.6083,  0.0065, -0.5868,  0.0396, -0.5643,  0.0731,
    -0.5372,  0.1041, -0.5033,  0.1259, -0.4662,  0.1406,
    -0.4275,  0.1493, -0.3881,  0.1530, -0.3487,  0.1526,
    -0.3095,  0.1488, -0.2708,  0.1421, -0.2319,  0.1342,
    -0.1943,  0.1217, -0.1600,  0.1025, -0.1290,  0.0785,
    -0.1012,  0.0509, -0.0764,  0.0206, -0.0547, -0.0120,
    -0.0378, -0.0472, -0.0324, -0.0857, -0.0389, -0.1241,
    -0.0546, -0.1599, -0.0814, -0.1876, -0.1193, -0.1964,
    -0.1582, -0.1935, -0.1931, -0.1769, -0.2157, -0.1453,
    -0.2290, -0.1085, -0.2327, -0.0697, -0.2240, -0.0317,
    -0.2064,  0.0033, -0.1853,  0.0362, -0.1613,  0.0672,
    -0.1350,  0.0961, -0.1051,  0.1213, -0.0706,  0.1397,
    -0.0332,  0.1512,  0.0053,  0.1580,  0.0442,  0.1624,
    0.0833,  0.1636,  0.1224,  0.1615,  0.1613,  0.1565,
    0.1999,  0.1500,  0.2378,  0.1402,  0.2749,  0.1279,
    0.3118,  0.1147,  0.3487,  0.1015,  0.3858,  0.0892,
    0.4236,  0.0787,  0.4621,  0.0715,  0.5012,  0.0702,
    0.5398,  0.0766,  0.5768,  0.0890,  0.6123,  0.1055,
    0.6466,  0.1244,  0.6805,  0.1440,  0.7147,  0.1630,
    0.7500,  0.1800
}, {
    -0.7500,  0.0000, -0.7033,  0.0195, -0.6569,  0.0399,
    -0.6104,  0.0600, -0.5634,  0.0789, -0.5155,  0.0954,
    -0.4667,  0.1089, -0.4174,  0.1206, -0.3676,  0.1299,
    -0.3174,  0.1365, -0.2669,  0.1398, -0.2162,  0.1391,
    -0.1658,  0.1347, -0.1157,  0.1271, -0.0661,  0.1169,
    -0.0170,  0.1046,  0.0316,  0.0903,  0.0791,  0.0728,
    0.1259,  0.0534,  0.1723,  0.0331,  0.2188,  0.0129,
    0.2656, -0.0064,  0.3122, -0.0263,  0.3586, -0.0466,
    0.4052, -0.0665,  0.4525, -0.0847,  0.5007, -0.1002,
    0.5497, -0.1130,  0.5991, -0.1240,  0.6491, -0.1325,
    0.6994, -0.1380,  0.7500, -0.1400,
// The rest of this is actually copied from above to make a continuous path
                                        -0.2157, -0.1453,
    -0.2290, -0.1085, -0.2327, -0.0697, -0.2240, -0.0317,
    -0.2064,  0.0033, -0.1853,  0.0362, -0.1613,  0.0672,
    -0.1350,  0.0961, -0.1051,  0.1213, -0.0706,  0.1397,
    -0.0332,  0.1512,  0.0053,  0.1580,  0.0442,  0.1624,
    0.0833,  0.1636,  0.1224,  0.1615,  0.1613,  0.1565,
    0.1999,  0.1500,  0.2378,  0.1402,  0.2749,  0.1279,
    0.3118,  0.1147,  0.3487,  0.1015,  0.3858,  0.0892,
    0.4236,  0.0787,  0.4621,  0.0715,  0.5012,  0.0702,
    0.5398,  0.0766,  0.5768,  0.0890,  0.6123,  0.1055,
    0.6466,  0.1244,  0.6805,  0.1440,  0.7147,  0.1630,
    0.7500,  0.1800
}};


static const float WIND_OFFSETS[2][2] = {
    {0.36, 0.11},
    {0.56, 0.16}
};


float X_ = 0.0;
float Y_ = 0.0;

uint8_t MODE = 0;
int last_millis = 0;
bool do_animation = true;

float sun_div = 24000;
float moon_div = 6000;
float cloud_div = 200000;
float puff_size = 1; // Larger value = smaller cloud
float STROKE = 0.04;
float cloud_puffs = 7;
float sleet_multi = 10;
float sleet_line = 3;
float sleet_start = -10;
float rain_multi = 5;
float rain_start = -5;
float snow_div = 3000;
float rain_div = 1350;
float sleet_div = 750;
float snow_flakes = 3;
float fog_div = 10000;
float fog_line = 0.9;
float wave_div = 2500;

uint8_t wind_counter = 0;

void setup() {
    Particle.function("function", fnRouter);

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)

    display.setTextSize(2);
	display.setTextColor(WHITE);

	display.clearDisplay();
}


void loop() {
    int now_millis = millis();

    if(now_millis!=last_millis) {
        doMode(now_millis);

        last_millis = now_millis;
    }
}


void doMode(int t) {
    if(!do_animation)
        t = 1;

    display.clearDisplay();

    switch(MODE) {
        case 1: // Moon
            display.setCursor(0,0);
	        display.println("ClearNight");

            moon(t, W*0.5, H*0.5, S, S*STROKE);
            break;

        case 2: // Partly cloudy day
            display.setCursor(0,0);
	        display.println("PartCloudy       Day");

            sun(t, W*0.625, H*0.375, S*0.75, S*STROKE);
            cloud(t, W*0.375, H*0.625, S*0.75, S*STROKE);
            break;

        case 3: // Partly cloudy night
            display.setCursor(0,0);
	        display.println("PartCloudy     Night");

            moon(t, W*0.667, H*0.375, S*0.75, S*STROKE);
            cloud(t, W*0.375, H*0.625, S*0.75, S*STROKE);
            break;

        case 4: // Cloudy
            display.setCursor(0,0);
	        display.println("Cloudy");

            cloud(t, W*0.5, H*0.5, S, S*STROKE);
            break;

        case 5: // Rain
            display.setCursor(0,0);
	        display.println("Rain");

            rain(t, W*0.5, H*0.37, S*0.9, S*STROKE);
            cloud(t, W*0.5, H*0.37, S*0.9, S*STROKE);
            break;

        case 6: // Sleet
            display.setCursor(0,0);
	        display.println("Sleet");

            sleet(t, W*0.5, H*0.37, S*0.9, S*STROKE);
            cloud(t, W*0.5, H*0.37, S*0.9, S*STROKE);
            break;

        case 7: // Snow
            display.setCursor(0,0);
	        display.println("Snow");

            snow(t, W*0.5, H*0.37, S*0.9, S*STROKE);
            cloud(t, W*0.5, H*0.37, S*0.9, S*STROKE);
            break;

        case 8: // Wind
            display.setCursor(0,0);
	        display.println("Windy");

            // swoosh0(t, W*0.5, H*0.5, S, S*STROKE, 0, 2);    // Curly wave
            // swoosh1(t, W*0.5, H*0.5, S, S*STROKE, 1, 2);    // Bottom wave

            // if(wind_counter<128)
                swoosh(t, W*0.5, H*0.5, S, S*STROKE, 0, 2);
            // else
                // swoosh(t, W*0.5, H*0.5, S, S*STROKE, 1, 2);

            wind_counter++;

            if(wind_counter==255)
                wind_counter = 0;

            break;

        case 9: // Fog
            display.setCursor(0,0);
	        display.println("Foggy");

            fog_wrapper(t);
            break;

        default: // Sun
            display.setCursor(0,0);
	        display.println("Sunny");

            sun(t, W*0.5, H*0.5, S, S*STROKE);
    }

    display.display();
}


int fnRouter(String command) {
    command.trim();
	command.toUpperCase();

	int retval = -1;

    if(command.equals("SUN")) {
        MODE = 0;
        retval = MODE;

	} else if(command.equals("MOON")) {
	    MODE = 1;
	    retval = MODE;

    } else if(command.equals("PARTLY_CLOUDY_DAY")) {
        MODE = 2;
        retval = MODE;

    } else if(command.equals("PARTLY_CLOUDY_NIGHT")) {
        MODE = 3;
        retval = MODE;

    } else if(command.equals("CLOUDY")) {
        MODE = 4;
        retval = MODE;

    } else if(command.equals("RAIN")) {
        MODE = 5;
        retval = MODE;

    } else if(command.equals("SLEET")) {
        MODE = 6;
        retval = MODE;

    } else if(command.equals("SNOW")) {
        MODE = 7;
        retval = MODE;

    } else if(command.equals("WIND")) {
        MODE = 8;
        retval = MODE;

    } else if(command.equals("FOG")) {
        MODE = 9;
        retval = MODE;

    } else if(command.startsWith("MODE=")) {
        MODE = command.substring(5).toInt();
        retval = MODE;

    } else if(command.startsWith("STROKE=")) {
        STROKE = command.substring(7).toFloat();
        retval = STROKE;

    } else if(command.startsWith("SUN_DIV=")) {
        sun_div = command.substring(8).toFloat();
        retval = sun_div;

    } else if(command.startsWith("MOON_DIV=")) {
        moon_div = command.substring(9).toFloat();
        retval = moon_div;

    } else if(command.startsWith("CLOUD_DIV=")) {
        cloud_div = command.substring(10).toFloat();
        retval = cloud_div;

    } else if(command.startsWith("PUFF_SIZE=")) {
        puff_size = command.substring(10).toFloat();
        retval = puff_size;

    } else if(command.startsWith("CLOUD_PUFFS=")) {
        cloud_puffs = command.substring(12).toFloat();
        retval = cloud_puffs;

    } else if(command.startsWith("SLEET_MULTI=")) {
        sleet_multi = command.substring(12).toFloat();
        retval = sleet_multi;

    } else if(command.startsWith("SLEET_LINE=")) {
        sleet_line = command.substring(11).toFloat();
        retval = sleet_line;

    } else if(command.startsWith("SLEET_START=")) {
        sleet_start = command.substring(12).toFloat();
        retval = sleet_start;

    } else if(command.startsWith("RAIN_MULTI=")) {
        rain_multi = command.substring(11).toFloat();
        retval = rain_multi;

    } else if(command.startsWith("RAIN_START=")) {
        rain_start = command.substring(11).toFloat();
        retval = rain_start;

    } else if(command.startsWith("SLEET_DIV=")) {
        sleet_div = command.substring(10).toFloat();
        retval = sleet_div;

    } else if(command.startsWith("RAIN_DIV=")) {
        rain_div = command.substring(9).toFloat();
        retval = rain_div;

    } else if(command.startsWith("SNOW_DIV=")) {
        snow_div = command.substring(9).toFloat();
        retval = snow_div;

    } else if(command.startsWith("SNOW_FLAKES=")) {
        snow_flakes = command.substring(12).toFloat();
        retval = snow_flakes;

    } else if(command.startsWith("FOG_DIV=")) {
        fog_div = command.substring(8).toFloat();
        retval = fog_div;

    } else if(command.startsWith("FOG_LINE=")) {
        fog_line = command.substring(9).toFloat();
        retval = fog_line;

    } else if(command.startsWith("WAVE_DIV=")) {
        wave_div = command.substring(9).toFloat();
        retval = wave_div;

    }


    if(retval>0)
        doMode(millis());


	return retval;
}


// GOOD (enough for now)
void fog_wrapper(float t) {
    float k = S*STROKE;

    cloud(t, W*0.5, H*0.32, S*0.75, k);

    t /= fog_div;

    float a = cos((t) * TAU) * S * 0.02,
          b = cos((t+0.25) * TAU) * S * 0.02,
          c = cos((t+0.50) * TAU) * S * 0.02,
          d = cos((t+0.75) * TAU) * S * 0.02,
          n = H * 0.936,
          e = (n-k * 0.5) + 0.5,
          f = (n-k * 2.5) + 0.5;

    line(a+W*0.2+k*0.5, e, (b+W*0.8-k*0.5)*fog_line, e);
    line(c+W*0.2+k*0.5, f, (d+W*0.8-k*0.5)*fog_line, f);
}


void circle(float x, float y, float r) {
    display.fillCircle(x+W_OFFSET, y+H_OFFSET, r, WHITE);
}


void circle(float x, float y, float r, uint16_t color) {
    display.fillCircle(x+W_OFFSET, y+H_OFFSET, r, color);
}


void arc(float x, float y, float r, float start, float end) {
    display.drawArc(x+W_OFFSET, y+H_OFFSET, r, start, end, WHITE);
}


void line(int ax, int ay, int bx, int by) {
    display.drawLine(ax+W_OFFSET, ay+H_OFFSET, bx+W_OFFSET, by+H_OFFSET, WHITE);
}


void moveTo(float x, float y) {
    X_ = x;
    Y_ = y;
}


void lineTo(float x, float y) {
    line(X_, Y_, x, y);

    X_ = x;
    Y_ = y;
}


// GOOD
void puff(float t, float cx, float cy, float rx, float ry, float rmin, float rmax, uint16_t color) {
    float c = cos(t*TAU)/puff_size,
          s = sin(t*TAU)/puff_size;

    rmax -= rmin;

    circle(
        cx - s * rx,
        cy + c * ry + rmax * 0.5,
        rmin + (1 - c * 0.5) * rmax,
        color
    );
}


// GOOD
void puffs(float t, float cx, float cy, float rx, float ry, float rmin, float rmax, uint16_t color) {
    for(int i=cloud_puffs; i--; ) {
        puff(t/i*cloud_puffs+i/cloud_puffs, cx, cy, rx, ry, rmin, rmax, color);
    }
}


// GOOD
void puffs(float t, float cx, float cy, float rx, float ry, float rmin, float rmax) {
    puffs(t, cx, cy, rx, ry, rmin, rmax, WHITE);
}


// GOOD (enough for now)
void cloud(float t, float cx, float cy, float cw, float s) {
    t /= cloud_div;

    float a = cw * 0.21/puff_size,
          b = cw * 0.12/puff_size,
          c = cw * 0.24/puff_size,
          d = cw * 0.28/puff_size;

    puffs(t, cx, cy, a, b, c, d, WHITE);
    puffs(t, cx, cy, a, b, c-s, d-s, BLACK);
}


// GOOD
void sun(float t, float cx, float cy, float cw, float s) {
    t /= sun_div;

    float a = cw * 0.25 - s * 0.5,
          b = cw * 0.32 + s * 0.5,
          c = cw * 0.50 - s * 0.5,
          i, p, cosine, sine;

    circle(cx, cy, a);

    for(i=8; i--; ) {
        p = (t+i/8) * TAU;
        cosine = cos(p);
        sine = sin(p);

        line(
            cx + cosine * b,
            cy + sine * b,
            cx + cosine * c,
            cy + sine * c
        );
    }
}


// GOOD
void moon(float t, float cx, float cy, float cw, float s) {
    t /= moon_div;

    float a = cw * 0.29 - s * 0.5,
          b = cw * 0.05,
          c = cos(t*TAU),
          p = c * TAU / -16;

    cx += c * b;

    circle(cx, cy, a);
    circle(cx + cos(p) * a * TWO_OVER_SQRT_2,  cy + sin(p) * a * TWO_OVER_SQRT_2, a, BLACK);
}


// GOOD (enough for now)
void rain(float t, float cx, float cy, float cw, float s) {
    t /= rain_div;

    float a = cw * 0.16,
          b = TAU * 11 / 12,
          c = TAU * 7 / 12,
          p, x, y;

    for(int i=4; i--; ) {
        p = fmod((t + i / 4), 1);
        x = cx + ((i - 1.5) / 1.5) * ((i == 1 || i == 2) ? -1 : 1) * a;
        //y = cy + p * p * cw;
        y = (cy + p * p * cw + i*rain_multi)-rain_start;

        circle(x, y, s, WHITE);
    }
}


// GOOD (enough for now)
void sleet(float t, float cx, float cy, float cw, float s) {
    t /= sleet_div;

    float a = cw * 0.1875,
          b = TAU * 11 / 12,
          c = TAU * 7 / 12,
          p, x, y;

    // 4 = 4th
    // 3 = 2nd
    // 2 = 3rd
    // 1 = 1st
    for(int i=4; i--; ) {
        p = fmod((t + i/4), 1);
        x = (cx + ((i - 1.5) / 1.5) * ((i==1 || i==2 ? -1 : 1) * a) + 0.5); // horizontal spacing
        y = (cy + p * cw + i*sleet_multi)-sleet_start;

        line(x, y-s*sleet_line, x, y+s*sleet_line); // length
    }
}


// GOOD
void snow(float t, float cx, float cy, float cw, float s) {
    t /= snow_div;

    float a = cw * 0.16,
          b = s * 0.75,
          u = t * TAU * 0.7,
          ux = cos(u) * b,
          uy = sin(u) * b,
          v = u + TAU / 3,
          vx = cos(v) * b,
          vy = sin(v) * b,
          w = u + TAU * 2 / 3,
          wx = cos(w) * b,
          wy = sin(w) * b,
          p, x, y;

    for(float i=snow_flakes; i--; ) {
        p = fmod((t + i / snow_flakes), 1);
        x = cx + sin((p + i / snow_flakes) * TAU) *a;
        y = cy + p * cw + (i*snow_flakes);

        line(x-ux, y-uy, x+ux, y+uy);
        line(x-vx, y-vy, x+vx, y+vy);
        line(x-wx, y-wy, x+wx, y+wy);
    }
}


void fogbank(float t, float cx, float cy, float cw, float s) {
    t /= fog_div;

    float a = cw * 0.25,
          b = cw * 0.06,
          c = cw * 0.21,
          d = cw * 0.28;

    // puffs(t, cx, cy, a, b, c, d);
    // puffs(t, cx, cy, a, b, c-s, d-s);
}


void leaf(float t, float x, float y, float cw, float s) {
    float a = cw / 8,
          b = a / 3,
          c = 2 * b,
          d = fmod(t, 1) * TAU,
          e = cos(d),
          f = sin(d);

    // arc(x, y, a, d, d+PI); // Makes no difference?
    // circle(x, y, a, WHITE);
    arc(x - b * e, y - b * f, c, d + PI, d); // leaf body
    // circle(x-b * e, y - b * f, c, BLACK);
    arc(x + c * e, y + c * f, b, d + PI, d);
}


// void swoosh0(float t, float cx, float cy, float cw, float s, int index, int total) {
//     t /= wave_div;

//     int path_size = (sizeof(WIND_PATHS1)/sizeof(float));

//     float a = fmod(t + index - WIND_OFFSETS[0][0], total),
//           c = fmod(t + index - WIND_OFFSETS[0][1], total),
//           e = fmod(t + index, total),
//           b, d, f, i;

//     if(a<1) {
//         a *= path_size / 2 -1;
//         b  = (int)a;
//         a -= b;
//         b *= 2;
//         b += 2;

//         moveTo(
//             cx + (WIND_PATHS0[(int)b - 2] * (1 - a) + WIND_PATHS0[(int)b] * a) * cw,
//             cy + (WIND_PATHS0[(int)b - 1] * (1 - a) + WIND_PATHS0[(int)b+1] * a) * cw
//         );

//         if(c<1) {
//             c *= path_size / 2 - 1;
//             d  = (int)c;
//             c -= d;
//             d *= 2;
//             d += 2;

//             for(i==b; i!=d; i +=2) {
//                 lineTo(
//                     cx + WIND_PATHS0[(int)i] * cw,
//                     cy + WIND_PATHS0[(int)i+1] * cw
//                 );
//             }

//             lineTo(
//                 cx + (WIND_PATHS0[(int)d-2] * (1-c) + WIND_PATHS0[(int)d] * c) * cw,
//                 cy + (WIND_PATHS0[(int)d-1] * (1-c) + WIND_PATHS0[(int)d+1] * c) *cw
//             );
//         } else {
//             for(i=b; i!=path_size; i+=2) {
//                 lineTo(
//                     cx + WIND_PATHS0[(int)i] * cw,
//                     cy + WIND_PATHS0[(int)i + 1] *cw
//                 );
//             }
//         }
//     } else if(c<1) {
//         c *= path_size / 2 - 1;
//         d  = (int)c;
//         c -= d;
//         d *= 2;
//         d += 2;

//         moveTo(
//             cx + WIND_PATHS0[0] * cw,
//             cy + WIND_PATHS0[1] * cw
//         );

//         for(i=2; i!=d; i+=2) {
//             lineTo(
//                 cx + WIND_PATHS0[(int)i] * cw,
//                 cy + WIND_PATHS0[(int)i + 1] * cw
//             );
//         }

//         lineTo(
//             cx + (WIND_PATHS0[(int)d - 2] * (1 - c) + WIND_PATHS0[(int)d    ] * c) * cw,
//             cy + (WIND_PATHS0[(int)d - 1] * (1 - c) + WIND_PATHS0[(int)d + 1] * c) * cw
//         );
//     }


//     if(e<1) {
//         e *= path_size / 2 - 1;
//         f  = (int)e;
//         e -= f;
//         f *= 2;
//         f += 2;

//         // leaf(
//         //     t,
//         //     cx + (WIND_PATHS0[(int)f-2] * (1-e) + WIND_PATHS0[(int)f]*e) * cw,
//         //     cy + (WIND_PATHS0[(int)f-1] * (1-e) + WIND_PATHS0[(int)f+1]*e) * cw,
//         //     cw,
//         //     s
//         // );
//     }
// }


// void swoosh1(float t, float cx, float cy, float cw, float s, int index, int total) {
//     t /= wave_div;

//     int path_size = (sizeof(WIND_PATHS1)/sizeof(float));

//     float a = fmod(t + index - WIND_OFFSETS[1][0], total),
//           c = fmod(t + index - WIND_OFFSETS[1][1], total),
//           e = fmod(t + index, total),
//           b, d, f, i;

//     if(a<1) {
//         a *= path_size / 2 -1;
//         b  = (int)a;
//         a -= b;
//         b *= 2;
//         b += 2;

//         moveTo(
//             cx + (WIND_PATHS1[(int)b - 2] * (1 - a) + WIND_PATHS1[(int)b] * a) * cw,
//             cy + (WIND_PATHS1[(int)b - 1] * (1 - a) + WIND_PATHS1[(int)b+1] * a) * cw
//         );

//         if(c<1) {
//             c *= path_size / 2 - 1;
//             d  = (int)c;
//             c -= d;
//             d *= 2;
//             d += 2;

//             for(i==b; i!=d; i +=2) {
//                 lineTo(
//                     cx + WIND_PATHS1[(int)i] * cw,
//                     cy + WIND_PATHS1[(int)i+1] * cw
//                 );
//             }

//             lineTo(
//                 cx + (WIND_PATHS1[(int)d-2] * (1-c) + WIND_PATHS1[(int)d] * c) * cw,
//                 cy + (WIND_PATHS1[(int)d-1] * (1-c) + WIND_PATHS1[(int)d+1] * c) *cw
//             );
//         } else {
//             for(i=b; i!=path_size; i+=2) {
//                 lineTo(
//                     cx + WIND_PATHS1[(int)i] * cw,
//                     cy + WIND_PATHS1[(int)i + 1] *cw
//                 );
//             }
//         }
//     } else if(c<1) {
//         c *= path_size / 2 - 1;
//         d  = (int)c;
//         c -= d;
//         d *= 2;
//         d += 2;

//         moveTo(
//             cx + WIND_PATHS1[0] * cw,
//             cy + WIND_PATHS1[1] * cw
//         );

//         for(i=2; i!=d; i+=2) {
//             lineTo(
//                 cx + WIND_PATHS1[(int)i] * cw,
//                 cy + WIND_PATHS1[(int)i + 1] * cw
//             );
//         }

//         lineTo(
//             cx + (WIND_PATHS1[(int)d - 2] * (1 - c) + WIND_PATHS1[(int)d    ] * c) * cw,
//             cy + (WIND_PATHS1[(int)d - 1] * (1 - c) + WIND_PATHS1[(int)d + 1] * c) * cw
//         );
//     }


//     if(e<1) {
//         e *= path_size / 2 - 1;
//         f  = (int)e;
//         e -= f;
//         f *= 2;
//         f += 2;

//         // leaf(
//         //     t,
//         //     cx + (WIND_PATHS1[(int)f-2] * (1-e) + WIND_PATHS1[(int)f]*e) * cw,
//         //     cy + (WIND_PATHS1[(int)f-1] * (1-e) + WIND_PATHS1[(int)f+1]*e) * cw,
//         //     cw,
//         //     s
//         // );
//     }
// }


void swoosh(float t, float cx, float cy, float cw, float s, int index, int total) {
    t /= wave_div;

    int path_size = (sizeof(WIND_PATHS[index])/sizeof(float));

    float a = fmod(t + index - WIND_OFFSETS[index][0], total),
          c = fmod(t + index - WIND_OFFSETS[index][1], total),
          e = fmod(t + index, total),
          b, d, f, i;

    if(a<1) {
        a *= path_size / 2 -1;
        b  = (int)a;
        a -= b;
        b *= 2;
        b += 2;

        moveTo(
            cx + (WIND_PATHS[index][(int)b - 2] * (1 - a) + WIND_PATHS[index][(int)b] * a) * cw,
            cy + (WIND_PATHS[index][(int)b - 1] * (1 - a) + WIND_PATHS[index][(int)b+1] * a) * cw
        );

        if(c<1) {
            c *= path_size / 2 - 1;
            d  = (int)c;
            c -= d;
            d *= 2;
            d += 2;

            for(i==b; i!=d; i +=2) {
                lineTo(
                    cx + WIND_PATHS[index][(int)i] * cw,
                    cy + WIND_PATHS[index][(int)i+1] * cw
                );
            }

            lineTo(
                cx + (WIND_PATHS[index][(int)d-2] * (1-c) + WIND_PATHS[index][(int)d] * c) * cw,
                cy + (WIND_PATHS[index][(int)d-1] * (1-c) + WIND_PATHS[index][(int)d+1] * c) *cw
            );
        } else {
            for(i=b; i!=path_size; i+=2) {
                lineTo(
                    cx + WIND_PATHS[index][(int)i] * cw,
                    cy + WIND_PATHS[index][(int)i + 1] *cw
                );
            }
        }
    } else if(c<1) {
        c *= path_size / 2 - 1;
        d  = (int)c;
        c -= d;
        d *= 2;
        d += 2;

        moveTo(
            cx + WIND_PATHS[index][0] * cw,
            cy + WIND_PATHS[index][1] * cw
        );

        for(i=2; i!=d; i+=2) {
            lineTo(
                cx + WIND_PATHS[index][(int)i] * cw,
                cy + WIND_PATHS[index][(int)i + 1] * cw
            );
        }

        lineTo(
            cx + (WIND_PATHS[index][(int)d - 2] * (1 - c) + WIND_PATHS[index][(int)d    ] * c) * cw,
            cy + (WIND_PATHS[index][(int)d - 1] * (1 - c) + WIND_PATHS[index][(int)d + 1] * c) * cw
        );
    }


    if(e<1) {
        e *= path_size / 2 - 1;
        f  = (int)e;
        e -= f;
        f *= 2;
        f += 2;

        leaf(
            t,
            cx + (WIND_PATHS[index][(int)f-2] * (1-e) + WIND_PATHS[index][(int)f]*e) * cw,
            cy + (WIND_PATHS[index][(int)f-1] * (1-e) + WIND_PATHS[index][(int)f+1]*e) * cw,
            cw,
            s
        );
    }
}