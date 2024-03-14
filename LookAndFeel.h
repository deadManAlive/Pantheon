#pragma once

#include <JuceHeader.h>

class PanLook : public LookAndFeel_V4 {
public:
    //==============================================================================
    enum class Origin {
        FromMin,
        FromMid,
    };

    enum class Channel {
        Left,
        Right,
    };

    //==============================================================================
    PanLook();
    PanLook(Origin, bool = false);
    PanLook(Origin, Channel, bool = false);

    void drawRotarySlider(Graphics&, int, int, int, int, float, float, float, Slider&) override;
    void drawLinearSlider(Graphics&, int, int, int, int, float, float, float, Slider::SliderStyle, Slider&) override;
    void drawGroupComponentOutline(Graphics&, int, int, const String&, const Justification&, GroupComponent&) override;
    void drawButtonBackground(Graphics&, Button&, const Colour&, bool, bool) override;
    void drawButtonText(Graphics&, TextButton&, bool, bool) override;
    //==============================================================================
    static Colour leftColour;
    static Colour rightColour;
    static Colour thumbColour;
    static Colour outlineColour;
    //==============================================================================
    float textH = 14.f;
private:
    Origin sliderOrigin;
    Channel sliderChannel;
    bool isReversed;
};