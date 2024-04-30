#include "PixelPainter.h"
#include "Adafruit_NeoPixel.h"

#include "fonts/pixel_regular.h"
#include "fonts/pixel_bold.h"

const uint8_t *lookupCharacter(char ascii, bool bold) {
    if (ascii < ' ' || ascii > '~') ascii = '?';
    if (bold) return pixel_text_bold[ascii - ' '];
    else return pixel_text_regular[ascii - ' '];
}
int lookupCharacterWidth(char ascii, bool bold) {
    if (ascii < ' ' || ascii > '~') ascii = '?';
    if (bold) return pixel_text_bold_letter_widths[ascii - ' '];
    else return pixel_text_regular_letter_widths[ascii - ' '];
}

int convertHex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 0xa;
    if (c >= 'A' && c <= 'F') return c - 'A' + 0xA;
    return -1;
}

class TextReader {
public: 
    const char *text;
    const uint32_t length;
    uint32_t pos = 0;

    char currentChar = '?';

    bool bold = false;
    bool italic = false;
    bool underline = false;
    bool strike = false;

    Color color = WHITE;
    Color highlight = CLEAR;

    TextReader(const char *text, uint32_t length) : text(text), length(length) {}

    bool next() {
        if (pos >= length) { return false; }
        if (text[pos] == '\\') {
            if (++pos < length)
            switch (text[pos]) {
                case '\\':
                    // do nothing I guess
                    break;
                case 'b':
                    if (++pos < length) {
                        if (text[pos] == '0') bold = true;
                        else if (text[pos] == '1') bold = false;
                    }
                    break;
                case 'i':
                    if (++pos < length) {
                        if (text[pos] == '0') italic = true;
                        else if (text[pos] == '1') italic = false;
                    }
                    break;
                case 'u':
                    if (++pos < length) {
                        if (text[pos] == '0') underline = true;
                        else if (text[pos] == '1') underline = false;
                    }
                    break;
                case 's':
                    if (++pos < length) {
                        if (text[pos] == '0') strike = true;
                        else if (text[pos] == '1') strike = false;
                    }
                    break;
                case 'm':
                    if (++pos < length) {
                        if (text[pos] == '0') {
                            color = WHITE;
                            ++pos; // skip semicolon
                        } else if (text[pos] == '#') {
                            uint32_t readColor = 0;
                            for (int i = 0; ++pos < length && i < 7; i++) {
                                if (text[pos] == '?') break;
                                int hexVal = convertHex(text[pos]);
                                if (hexVal == -1) break;
                                readColor |= hexVal << (i * 4);
                            }
                            color.r = readColor & 0xFF;
                            color.g = readColor >> 8 & 0xFF;
                            color.b = readColor >> 16 & 0xFF;
                            color.a = 0xFF;
                        } // TODO: support rgb
                    }
                    break;
                case 'h':
                    if (++pos < length) {
                        if (text[pos] == '0') {
                            highlight = CLEAR;
                            ++pos; // skip semicolon
                        } else if (text[pos] == '#') {
                            uint32_t readColor = 0;
                            for (int i = 0; ++pos < length && i < 7; i++) {
                                if (text[pos] == '?') break;
                                int hexVal = convertHex(text[pos]);
                                if (hexVal == -1) break;
                                readColor |= hexVal << (i * 4);
                            }
                            highlight.r = readColor & 0xFF;
                            highlight.g = readColor >> 8 & 0xFF;
                            highlight.b = readColor >> 16 & 0xFF;
                            highlight.a = readColor == 0 ? 0 : 255;
                        } // TODO: support rgb
                    }
                    break;
            }
        }
        if (pos < length) currentChar = text[pos];
        ++pos;
        return pos >= length;
    }
    operator bool() const { return pos >= length; }
};

void PixelPainter::setMessage(const char *message, uint32_t length) {
    if (length > MAX_MESSAGE_LENGTH) return;
    if (message != nullptr) {
        for (uint32_t i = 0; i < length; i++) m_message[i] = message[i];
    } else {
        length = 0;
    }
    // calculate pixel length so scrolling works correctly
    m_messageLength = length;
    m_messagePixels = 0;
    TextReader reader(message, length);
    while (reader.next()) {
        m_messagePixels += lookupCharacterWidth(reader.currentChar, reader.bold);
    }
    if (m_messagePixels <= 32) {
        m_scrollTime = 0;
        m_scrollOffset = (32 - m_messagePixels) / 2;
    } else {
        m_scrollTime = 200;
        m_scrollOffset = 32;
    }
    m_scrollTimestamp = millis();
    m_needsRefresh = true;
}
void PixelPainter::pushFrame(KeyFrame *frame) {
    if (m_firstFrame == nullptr) {
        m_firstFrame = frame;
        setCurrentFrame(frame);
    }
    if (m_lastFrame != nullptr) m_lastFrame->nextPtr = frame;
    m_lastFrame = frame;
}
void PixelPainter::pushColorFrame(Color color, uint32_t duration) {
    ColorFrame *frame = new ColorFrame{{nullptr, FrameType::COLOR, duration}, color};
    pushFrame(reinterpret_cast<KeyFrame*>(frame));
}
void PixelPainter::pushImageFrame(ColorFormat format, const uint8_t *data, uint32_t duration) {
    ImageFrame *frame = new ImageFrame{{nullptr, FrameType::IMAGE, duration}, format, data};
    pushFrame(reinterpret_cast<KeyFrame*>(frame));
}

void PixelPainter::clearAnimation() {
    if (m_firstFrame == nullptr) return;
    KeyFrame *frame = m_firstFrame;
    while (frame != m_lastFrame) {
        KeyFrame *old = frame;
        frame = old->nextPtr;
        delete old;
    }
    delete m_lastFrame;
    m_firstFrame = nullptr;
    m_currentFrame = nullptr;
    m_lastFrame = nullptr;
    m_playingAnimation = false;
    m_repeatAnimation = false;
    m_animationProgress = 0;
    m_needsRefresh = true;
}

void PixelPainter::update() {
    uint32_t now = millis();
    uint32_t delta = now - m_lastTimestamp;
    if (m_playingAnimation) { 
      m_animationProgress += delta;
      m_lastTimestamp = now;
    }
    if (m_currentFrame != nullptr && m_animationProgress > m_currentFrame->duration) {
        m_animationProgress -= m_currentFrame->duration;
        if (m_currentFrame == m_lastFrame) {
            stopAnimation();
            if (m_repeatAnimation) startAnimation();
        } else {
            setCurrentFrame(m_currentFrame->nextPtr);
        }
    }
    if (m_messageLength != 0 && m_scrollTime != 0 && now - m_scrollTimestamp >= m_scrollTime) {
        --m_scrollOffset;
        if (m_scrollOffset < -(int)m_messagePixels) m_scrollOffset = 32;
        m_needsRefresh = true;
    }

    if (m_needsRefresh) {
        renderBackground();
        renderFrame();
        renderText();
        m_neoPixel.show();
    }
}

void PixelPainter::renderText() {
    int pos = m_scrollOffset;
    TextReader reader(m_message, m_messageLength);
    while (reader.next() && pos < 32) {
        const uint8_t *data = lookupCharacter(reader.currentChar, reader.bold);
        int charWidth = lookupCharacterWidth(reader.currentChar, reader.bold);
        for (int col = 0; col < charWidth; col++) {
            if (pos >= 0 && pos) {
                // draw column
                for (int row = 0; row < 8; row++) {
                    int index = row * 8 + col;
                    uint8_t pixel = data[index];
                    if (pixel == 0) {
                        if (reader.highlight.a != 0) 
                            m_neoPixel.setPixelColor(index, reader.highlight.r, reader.highlight.g, reader.highlight.b);
                        else m_neoPixel.setPixelColor(index, 0, 0, 0);
                    } else {
                        m_neoPixel.setPixelColor(index, reader.color.r, reader.color.g, reader.color.b);
                        if (row == 7 && reader.underline) m_neoPixel.setPixelColor(index, reader.color.r, reader.color.g, reader.color.b);
                        if (row == 3 && reader.strike) m_neoPixel.setPixelColor(index, reader.color.r, reader.color.g, reader.color.b);
                    }
                }
            }
            ++pos;
        }
        ++pos; // spacing
    }
}

void PixelPainter::renderFrame() {
    if (m_currentFrame == nullptr) return;
    switch (m_currentFrame->type) {
        case FrameType::COLOR:
        {
            Color color = reinterpret_cast<ColorFrame*>(m_currentFrame)->color;
            if (color.a == 0) m_neoPixel.clear();
            else m_neoPixel.fill(Adafruit_NeoPixel::Color(color.r, color.g, color.b));
            break;
        }
        case FrameType::IMAGE:
        {
            ImageFrame *iframe = reinterpret_cast<ImageFrame*>(m_currentFrame);
            switch (iframe->format) {
                case ColorFormat::GRAYSCALE:
                {
                    for (int i = 0; i < 32*8; i++) {
                        uint8_t value = iframe->dataPtr[i];
                        m_neoPixel.setPixelColor(i, value, value, value);
                    }
                    break;
                }
                case ColorFormat::RGB:
                {
                    for (int i = 0; i < 32*8; i++) {
                        const uint8_t *rgb = iframe->dataPtr + i*3;
                        m_neoPixel.setPixelColor(i, rgb[0], rgb[1], rgb[2]);
                    }
                    break;
                }
            }
            break;
        }
    }
}

void PixelPainter::renderBackground() {
    if (m_backgroundColor.a == 0) m_neoPixel.clear();
    else m_neoPixel.fill(Adafruit_NeoPixel::Color(m_backgroundColor.r, m_backgroundColor.g, m_backgroundColor.b));
}