#ifndef PIXEL_PAINTER_H
#define PIXEL_PAINTER_H

#include <Adafruit_NeoPixel.h>

#define MAX_BRIGHTNESS          70
#define LED_COUNT               256

#define MAX_MESSAGE_LENGTH      256

struct Color {
    uint8_t r, g, b;
    uint8_t a = 255; // a will only be transparent if it is exactly 0
};

const Color WHITE = {255, 255, 255};
const Color RED = {255, 0, 0};
const Color YELLOW = {255, 255, 0};
const Color GREEN = {0, 255, 0};
const Color CYAN = {0, 255, 255};
const Color BLUE = {0, 0, 255};
const Color MAGENTA = {255, 0, 255};
const Color CLEAR = {0, 0, 0, 0};

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
    uint32_t duration; // milliseconds
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

    void setMessage(const char *message, uint32_t length); // text renders above animations

    void setTextScrollSpeed(float pixelsPerSecond) { m_scrollTime = (uint32_t)((1 / pixelsPerSecond) * 1000); }
    void setBackgroundColor(Color color) { m_backgroundColor = color; m_needsRefresh = true; } // sets color to show when no animation or text highlight
    void setBrightness(uint8_t brightness) { m_neoPixel.setBrightness(brightness); }

    void pushColorFrame(Color color, uint32_t duration);
    void pushImageFrame(ColorFormat format, const uint8_t *data, uint32_t duration);
    void clearAnimation();

    void setRepeatAnimation(bool repeat) { m_repeatAnimation = repeat; }
    void startAnimation() { m_playingAnimation = true; m_needsRefresh = true; m_lastTimestamp = millis(); }
    void pauseAnimation() { m_playingAnimation = false; }
    void stopAnimation() { m_playingAnimation = false; setCurrentFrame(m_firstFrame); }

    void clearAll() { clearAnimation(); setMessage(nullptr, 0); }

    void update();

private:
    void pushFrame(KeyFrame *frame);
    void setCurrentFrame(KeyFrame *frame) { m_currentFrame = frame; m_animationProgress = 0; m_needsRefresh = true; }
    void renderText();
    void renderFrame();
    void renderBackground();

    bool m_enabled = true;
    Color m_backgroundColor = CLEAR;
    char m_message[256];
    uint32_t m_messageLength = 0;
    uint32_t m_messagePixels = 0;
    uint32_t m_scrollTimestamp = 0;
    int32_t m_scrollOffset = 0;
    uint32_t m_scrollTime = 200; // milliseconds; 0 for no scroll
    
    KeyFrame *m_firstFrame = nullptr;
    KeyFrame *m_lastFrame = nullptr;
    KeyFrame *m_currentFrame = nullptr;
    uint32_t m_animationProgress = 0;
    uint32_t m_lastTimestamp = 0;
    bool m_repeatAnimation = false;
    bool m_playingAnimation = false;
    
    bool m_needsRefresh = false;

    //uint8_t m_brightness = 255; // 0 is off, 255 is normal

    uint8_t m_pin;
    Adafruit_NeoPixel m_neoPixel;
};

#endif