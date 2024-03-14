#include "LookAndFeel.h"
#include <utility>

PanLook::PanLook()
{
}

PanLook::PanLook(Origin origin, Channel channel, bool reversed)
    : sliderOrigin(origin)
    , sliderChannel(channel)
    , isReversed(reversed)
{   
}

PanLook::PanLook(Origin origin, bool reversed)
    : sliderOrigin(origin)
    , isReversed(reversed)
{
}

Colour PanLook::leftColour = Colours::goldenrod;
Colour PanLook::rightColour = Colours::indianred;
Colour PanLook::thumbColour = Colours::bisque;
Colour PanLook::outlineColour = Colours::linen;

void PanLook::drawRotarySlider(Graphics& g, int x, int y, int width, int height, float sliderPos, float rotaryStartAngle, float rotaryEndAngle, Slider& slider) {
    const auto outline = slider.findColour (Slider::rotarySliderOutlineColourId);

    Colour fill;
    float fillOriginAngle;

    if (sliderOrigin == Origin::FromMid) {
        fill = sliderPos > 0.5f ? rightColour : leftColour;
        fillOriginAngle = MathConstants<float>::twoPi;
    } else {
        fill = rightColour;
        fillOriginAngle = rotaryStartAngle;
    }

    const auto bounds = Rectangle<int> (x, y, width, height).toFloat().reduced (10);

    const auto radius = jmin (bounds.getWidth(), bounds.getHeight()) / 2.0f;
    const auto toAngle = rotaryStartAngle + sliderPos * (rotaryEndAngle - rotaryStartAngle);
    const auto lineW = jmin (8.0f, radius * 0.25f);
    const auto arcRadius = radius - lineW * 0.5f;

    Path backgroundArc;
    backgroundArc.addCentredArc (bounds.getCentreX(),
                                 bounds.getCentreY(),
                                 arcRadius,
                                 arcRadius,
                                 0.0f,
                                 rotaryStartAngle,
                                 rotaryEndAngle,
                                 true);

    g.setColour (outline);
    g.strokePath (backgroundArc, PathStrokeType (lineW, PathStrokeType::mitered, PathStrokeType::butt));

    if (slider.isEnabled())
    {
        Path valueArc;
        valueArc.addCentredArc (bounds.getCentreX(),
                                bounds.getCentreY(),
                                arcRadius,
                                arcRadius,
                                0.0f,
                                fillOriginAngle,
                                toAngle,
                                true);

        g.setColour (fill);
        g.strokePath (valueArc, PathStrokeType (lineW, PathStrokeType::mitered, PathStrokeType::butt));
    }

    Path thumbArc;

    const auto thumbArcStartAngle = toAngle - 0.1f;
    const auto thumbArcEndAngle = toAngle + 0.1f;
    
    thumbArc.addCentredArc(bounds.getCentreX(),
                           bounds.getCentreY(),
                           arcRadius - lineW * 1.5f,
                           arcRadius - lineW * 1.5f,
                           0.f,
                           thumbArcStartAngle,
                           thumbArcEndAngle,
                           true);

    g.setColour(thumbColour);
    g.strokePath(thumbArc, PathStrokeType(4.f * lineW, PathStrokeType::mitered, PathStrokeType::butt));
}

void PanLook::drawLinearSlider(Graphics& g,
                               int x,
                               int y,
                               int width,
                               int height,
                               float sliderPos,
                               float minSliderPos,
                               float maxSliderPos,
                               Slider::SliderStyle style,
                               Slider& slider) {
    
    auto trackWidth = jmin (8.0f, slider.isHorizontal() ? (float) height * 0.25f : (float) width * 0.25f);
    
    Point<float> startPoint (slider.isHorizontal() ? (float) x : (float) x + (float) width * 0.5f,
                             slider.isHorizontal() ? (float) y + (float) height * 0.5f : (float) (height + y));
    Point<float> endPoint (slider.isHorizontal() ? (float) (width + x) : startPoint.x,
                           slider.isHorizontal() ? startPoint.y : (float) y);

    if (isReversed) {
        std::swap(startPoint, endPoint);
    }

    Point<float> midPoint(slider.isHorizontal() ? (float) (x + width * 0.5) : startPoint.x,
                          slider.isHorizontal() ? startPoint.y : (float) (y + height * 0.5));
    Point<float> thumbPoint(slider.isHorizontal() ? sliderPos : startPoint.x,
                            slider.isHorizontal() ? startPoint.y : sliderPos);

    Path backgroundTrack;
    backgroundTrack.startNewSubPath(startPoint);
    backgroundTrack.lineTo(endPoint);
    g.setColour(slider.findColour(Slider::backgroundColourId));
    g.strokePath(backgroundTrack, {trackWidth, PathStrokeType::mitered, PathStrokeType::square});

    const auto fill = sliderChannel == Channel::Left ? leftColour : rightColour;

    Path valueTrack;
    auto psType = PathStrokeType::butt;
    if (sliderOrigin == Origin::FromMin) {
        valueTrack.startNewSubPath(startPoint);
        psType = PathStrokeType::square;
    } else {
        valueTrack.startNewSubPath(midPoint);
    }
    valueTrack.lineTo(thumbPoint);
    g.setColour(fill);
    g.strokePath(valueTrack, {trackWidth, PathStrokeType::mitered, psType});

    Path thumbTrack;

    const auto hoffset = jmin(8.f, (float) width * 0.02f);
    // Point<float> thumbStartPoint(startPoint.x, thumbPoint.y - (float)height * 0.02f);
    Point<float> thumbStartPoint(slider.isHorizontal() ? thumbPoint.x - hoffset : startPoint.x,
                                 slider.isHorizontal() ? startPoint.y : thumbPoint.y - (float) height * 0.02f);
    // Point<float> thumbEndPoint(startPoint.x, thumbPoint.y + (float)height * 0.02f);
    Point<float> thumbEndPoint(slider.isHorizontal() ? thumbPoint.x + hoffset : startPoint.x,
                               slider.isHorizontal() ? startPoint.y : thumbPoint.y + (float) height * 0.02f);

    thumbTrack.startNewSubPath(thumbStartPoint);
    thumbTrack.lineTo(thumbEndPoint);
    g.setColour(thumbColour);
    g.strokePath(thumbTrack, {2 * trackWidth, PathStrokeType::mitered, PathStrokeType::butt});

    ignoreUnused(minSliderPos, maxSliderPos, style);
}

void PanLook::drawGroupComponentOutline (Graphics& g, int width, int height,
                                                const String& text, const Justification& position,
                                                GroupComponent& group)
{
    const float indent = 3.0f;
    const float textEdgeGap = 4.0f;
    auto cs = 5.0f;

    Font f ("Lobster", textH, Font::plain);

    Path p;
    auto x = indent;
    auto y = f.getAscent() - 3.0f;
    auto w = jmax (0.0f, (float) width - x * 2.0f);
    auto h = jmax (0.0f, (float) height - y  - indent);
    cs = jmin (cs, w * 0.5f, h * 0.5f);
    auto cs2 = 2.0f * cs;

    auto textW = text.isEmpty() ? 0
                                : jlimit (0.0f,
                                          jmax (0.0f, w - cs2 - textEdgeGap * 2),
                                          (float) f.getStringWidth (text) + textEdgeGap * 2.0f);
    auto textX = cs + textEdgeGap;

    if (position.testFlags (Justification::horizontallyCentred))
        textX = cs + (w - cs2 - textW) * 0.5f;
    else if (position.testFlags (Justification::right))
        textX = w - cs - textW - textEdgeGap;

    p.startNewSubPath (x + textX + textW, y);
    p.lineTo (x + w - cs, y);

    p.addArc (x + w - cs2, y, cs2, cs2, 0, MathConstants<float>::halfPi);
    p.lineTo (x + w, y + h - cs);

    p.addArc (x + w - cs2, y + h - cs2, cs2, cs2, MathConstants<float>::halfPi, MathConstants<float>::pi);
    p.lineTo (x + cs, y + h);

    p.addArc (x, y + h - cs2, cs2, cs2, MathConstants<float>::pi, MathConstants<float>::pi * 1.5f);
    p.lineTo (x, y + cs);

    p.addArc (x, y, cs2, cs2, MathConstants<float>::pi * 1.5f, MathConstants<float>::twoPi);
    p.lineTo (x + textX, y);

    auto alpha = group.isEnabled() ? 1.0f : 0.5f;

    g.setColour (group.findColour (GroupComponent::outlineColourId)
                    .withMultipliedAlpha (alpha));

    g.strokePath (p, PathStrokeType (2.0f));

    g.setColour (group.findColour (GroupComponent::textColourId)
                    .withMultipliedAlpha (alpha));
    g.setFont (f);
    g.drawText (text,
                roundToInt (x + textX), 0,
                roundToInt (textW),
                roundToInt (textH),
                Justification::centred, true);
}

void PanLook::drawButtonBackground(Graphics& g,
                                           Button& button,
                                           const Colour& backgroundColour,
                                           bool shouldDrawButtonAsHighlighted,
                                           bool shouldDrawButtonAsDown)
{
    auto cornerSize = 4.0f;
    auto bounds = button.getLocalBounds().toFloat().reduced (0.5f, 0.5f);

    auto baseColour = backgroundColour;

    baseColour = button.hasKeyboardFocus(true)
                    ? baseColour.withMultipliedBrightness(1.3f)
                    : baseColour.withMultipliedSaturation(0.9f);

    baseColour = button.getToggleState()
                    ? rightColour
                    : baseColour;
                    
    baseColour = button.isEnabled()
                    ? baseColour.withMultipliedAlpha(1.0f)
                    : baseColour.withMultipliedAlpha(0.5f);

    if (shouldDrawButtonAsDown || shouldDrawButtonAsHighlighted) {
        baseColour = shouldDrawButtonAsDown
                        ? baseColour.contrasting(0.2f)
                        : baseColour.contrasting(0.5f);
    }


    g.setColour (baseColour);

    auto flatOnLeft   = button.isConnectedOnLeft();
    auto flatOnRight  = button.isConnectedOnRight();
    auto flatOnTop    = button.isConnectedOnTop();
    auto flatOnBottom = button.isConnectedOnBottom();

    if (flatOnLeft || flatOnRight || flatOnTop || flatOnBottom)
    {
        Path path;
        path.addRoundedRectangle (bounds.getX(), bounds.getY(),
                                  bounds.getWidth(), bounds.getHeight(),
                                  cornerSize, cornerSize,
                                  ! (flatOnLeft  || flatOnTop),
                                  ! (flatOnRight || flatOnTop),
                                  ! (flatOnLeft  || flatOnBottom),
                                  ! (flatOnRight || flatOnBottom));

        g.fillPath (path);

        g.setColour (outlineColour);
        g.strokePath (path, PathStrokeType (2.0f));
    }
    else
    {
        g.fillRoundedRectangle (bounds, cornerSize);

        g.setColour (outlineColour);
        g.drawRoundedRectangle (bounds, cornerSize, 2.0f);
    }
}

void PanLook::drawButtonText (Graphics& g, TextButton& button,
                                     bool /*shouldDrawButtonAsHighlighted*/, bool /*shouldDrawButtonAsDown*/)
{
    const auto isToggled = button.getToggleState();
    
    Font font (getTextButtonFont (button, button.getHeight()));
    
    if (isToggled) {
        font.setBold(true);
    }

    g.setFont (font);

    auto buttonTextColour = button.findColour(button.getToggleState()
                            ? TextButton::textColourOnId
                            : TextButton::textColourOffId);

    buttonTextColour = isToggled
                        ? thumbColour
                        : buttonTextColour;

    buttonTextColour = button.isEnabled()
                    ? buttonTextColour.withMultipliedAlpha(1.0f)
                    : buttonTextColour.withMultipliedAlpha(0.5f);

    g.setColour (buttonTextColour);

    const int yIndent = jmin (4, button.proportionOfHeight (0.3f));
    const int cornerSize = jmin (button.getHeight(), button.getWidth()) / 2;

    const int fontHeight = roundToInt (font.getHeight() * 0.6f);
    const int leftIndent  = jmin (fontHeight, 2 + cornerSize / (button.isConnectedOnLeft() ? 4 : 2));
    const int rightIndent = jmin (fontHeight, 2 + cornerSize / (button.isConnectedOnRight() ? 4 : 2));
    const int textWidth = button.getWidth() - leftIndent - rightIndent;

    if (textWidth > 0)
        g.drawFittedText (button.getButtonText(),
                          leftIndent, yIndent, textWidth, button.getHeight() - yIndent * 2,
                          Justification::centred, 2);
}