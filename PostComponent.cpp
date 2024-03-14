#include "PostComponent.h"

PostComponent::PostComponent(AudioPluginAudioProcessor& p, AudioProcessorValueTreeState& apvts)
    : processorRef(p)
    , parameters(apvts)
    , leftPanLook(PanLook::Origin::FromMin, PanLook::Channel::Left, true)
    , rightPanLook(PanLook::Origin::FromMin, PanLook::Channel::Right)
    , leftPostPanSlider(Slider::LinearHorizontal, Slider::NoTextBox)
    , rightPostPanSlider(Slider::LinearHorizontal, Slider::NoTextBox)
{
    leftPostPanSlider.setLookAndFeel(&leftPanLook);
    addAndMakeVisible(leftPostPanSlider);
    leftPostPanAttachment.reset(new SliderAttachment(parameters, "leftPan", leftPostPanSlider));

    rightPostPanSlider.setLookAndFeel(&rightPanLook);
    addAndMakeVisible(rightPostPanSlider);
    rightPostPanAttachment.reset(new SliderAttachment(parameters, "rightPan", rightPostPanSlider));
    
    border.setText("Post");
    border.setLookAndFeel(&leftPanLook);
    border.setEnabled(false);
    addAndMakeVisible(border);
}

PostComponent::~PostComponent() {}

void PostComponent::paint(juce::Graphics& g) {
    g.fillAll (getLookAndFeel().findColour (ResizableWindow::backgroundColourId));

    const auto bounds = getLocalBounds().toFloat();
    const float height = bounds.getHeight();
    const float width = bounds.getWidth();

    const auto r = leftPostPanSlider.getNormalisableRange();
    const auto offset = width / 12.f;

    for (double s = r.start; s <= r.end; s += 1.) {
        g.setColour(Colours::linen.withAlpha(0.5f + 0.25f * (float)std::abs(s) / (float)std::abs(r.start)));
        const auto pos = leftPostPanSlider.getPositionOfValue(s);
        g.drawLine({Point<float>{pos + offset, 0.2f * height}, Point<float>{pos + offset, 0.8f * height}}, 2.f);
    }
}

void PostComponent::resized() {
    border.setBounds(getLocalBounds());

    Grid grid;

    using Track = Grid::TrackInfo;
    using Fr = Grid::Fr;

    grid.templateRows = {
        Track(Fr(1)),
        Track(Fr(1)),
    };
    grid.templateColumns = {
        Track(Fr(1)),
        Track(Fr(10)),
        Track(Fr(1)),
    };

    grid.items = {
        GridItem(leftPostPanSlider).withArea(1, 2),
        GridItem(rightPostPanSlider).withArea(2, 2),
    };

    grid.performLayout(getLocalBounds().reduced(0, 8));
}
