#ifndef PIXEL_PAINTER_H
#define PIXEL_PAINTER_H

#include <Adafruit_NeoPixel.h>

#define MAX_BRIGHTNESS          70
#define LED_COUNT               256

#include <RoveCommManifest.h>
#define MAX_MESSAGE_LENGTH      RC_COREBOARD_LEDTEXT_DATA_COUNT // 256
#define SCROLL_TIME             20000 // microseconds

struct Color {
    uint8_t r, g, b;
    uint8_t transparent;
};

const Color WHITE = {255, 255, 255};
const Color RED = {255, 0, 0};
const Color YELLOW = {255, 255, 0};
const Color GREEN = {0, 255, 0};
const Color CYAN = {0, 255, 255};
const Color BLUE = {0, 0, 255};
const Color MAGENTA = {255, 0, 255};
const Color CLEAR = {0, 0, 0, true};

enum FrameType {
    COLOR, IMAGE,  
};
enum ColorFormat {
    GRAYSCALE, RGB,
};

// old fashioned C inheritance
struct KeyFrame {
    KeyFrame *nextPtr;
    FrameType type;
    uint32_t duration; // microseconds
};
struct ColorFrame {
    KeyFrame frame;
    Color color;
};
struct ImageFrame {
    KeyFrame frame;
    ColorFormat format;
    const uint8_t *dataPtr;
};

class PixelPainter {
public:

    PixelPainter(uint8_t pin): m_pin(pin), m_neoPixel(Adafruit_NeoPixel(LED_COUNT, pin)) {}
    ~PixelPainter() { clearAnimation(); }

    void begin() { m_neoPixel.begin(); }

    void setPixelRGB(int x, int y, Color color) {
        if (color.transparent) return;
        int i = 256 - (8 * x) + (x % 2 == 0 ? -8 + y :  -y - 1);
        m_neoPixel.setPixelColor(i, color.r, color.g, color.b);
    }
    void setPixelGrayscale(int x, int y, uint8_t value) {
        int i = 256 - (8 * x) + (x % 2 == 0 ? -8 + y :  -y - 1);
        m_neoPixel.setPixelColor(i, value, value, value);
    }

    void setMessage(const char *message, uint32_t length); // text renders above animations

    void setTextScrollSpeed(float pixelsPerSecond) { m_scrollTime = (uint32_t)((1 / pixelsPerSecond) * 1000); }
    void setBackgroundColor(Color color) { m_backgroundColor = color; m_needsRefresh = true; } // sets color to show when no animation or text highlight
    void setBrightness(uint8_t brightness) { m_neoPixel.setBrightness(constrain(brightness, 0, MAX_BRIGHTNESS)); }

    // duration in milliseconds
    void pushColorFrame(Color color, uint32_t duration);
    // duration in milliseconds
    void pushImageFrame(ColorFormat format, const uint8_t *data, uint32_t duration);
    void clearAnimation();

    void setRepeatAnimation(bool repeat) { m_repeatAnimation = repeat; }
    void startAnimation() { m_playingAnimation = true; m_needsRefresh = true; m_lastTimestamp = micros(); }
    void pauseAnimation() { m_playingAnimation = false; }
    void stopAnimation() { m_playingAnimation = false; setCurrentFrame(m_firstFrame); }

    void clearAll() { clearAnimation(); setMessage(nullptr, 0); setBackgroundColor(CLEAR); }

    void update();

private:
    void pushFrame(KeyFrame *frame);
    void setCurrentFrame(KeyFrame *frame) { m_currentFrame = frame; m_animationProgress = 0; m_needsRefresh = true; }
    void renderText();
    void renderFrame();
    void renderBackground();

    bool m_enabled = true;
    Color m_backgroundColor = CLEAR;
    char m_message[MAX_MESSAGE_LENGTH];
    uint32_t m_messageLength = 0;
    uint32_t m_messagePixels = 0;
    uint32_t m_scrollTimestamp = 0; // microseconds
    int32_t m_scrollOffset = 0; // how many pixels shifted to the left
    uint32_t m_scrollTime = SCROLL_TIME; // microseconds; 0 for no scroll
    
    KeyFrame *m_firstFrame = nullptr;
    KeyFrame *m_lastFrame = nullptr;
    KeyFrame *m_currentFrame = nullptr;
    uint32_t m_animationProgress = 0; // microseconds
    uint32_t m_lastTimestamp = 0; // microseconds
    bool m_repeatAnimation = false;
    bool m_playingAnimation = false;
    
    bool m_needsRefresh = false;

    //uint8_t m_brightness = 255; // 0 is off, 255 is normal

    uint8_t m_pin;
    Adafruit_NeoPixel m_neoPixel;
};

#endif
