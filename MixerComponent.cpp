#include "MixerComponent.h"
#include <cstdlib>

MixerComponent::MixerComponent(AudioPluginAudioProcessor& p, AudioProcessorValueTreeState& apvts)
    : processorRef(p)
    , parameters(apvts)
    , leftPanLook(PanLook::Origin::FromMid, PanLook::Channel::Left)
    , rightPanLook(PanLook::Origin::FromMid, PanLook::Channel::Right)
    , rightToLeftGainSlider(Slider::LinearVertical, Slider::NoTextBox)
    , leftPreGainSlider(Slider::LinearVertical, Slider::NoTextBox)
    , rightPreGainSlider(Slider::LinearVertical, Slider::NoTextBox)
    , leftToRightGainSlider(Slider::LinearVertical, Slider::NoTextBox)
{
    addAndMakeVisible(rightToLeftGainSlider);
    rightToLeftGainSlider.setLookAndFeel(&rightPanLook);
    rightToLeftGainAttachment.reset(new SliderAttachment(parameters, "rightToLeftGain", rightToLeftGainSlider));

    addAndMakeVisible(leftPreGainSlider);
    leftPreGainSlider.setLookAndFeel(&leftPanLook);
    leftPreGainAttachment.reset(new SliderAttachment(parameters, "leftPreGain", leftPreGainSlider));

    addAndMakeVisible(rightPreGainSlider);
    rightPreGainSlider.setLookAndFeel(&rightPanLook);
    rightPreGainAttachment.reset(new SliderAttachment(parameters, "rightPreGain", rightPreGainSlider));

    addAndMakeVisible(leftToRightGainSlider);
    leftToRightGainSlider.setLookAndFeel(&leftPanLook);
    leftToRightGainAttachment.reset(new SliderAttachment(parameters, "leftToRightGain", leftToRightGainSlider));
}

MixerComponent::~MixerComponent() {}

void MixerComponent::paint(juce::Graphics& g) {
    g.fillAll (getLookAndFeel().findColour (juce::ResizableWindow::backgroundColourId));

    const auto bounds = getLocalBounds().toFloat();
    const float width = bounds.getWidth();

    const auto r = rightToLeftGainSlider.getNormalisableRange();

    for (double s = r.start; s <= r.end; s += 1.) {
        g.setColour (Colours::linen.withAlpha(0.5f + 0.25f * (float)std::abs(s) / (float)std::abs(r.start)));
        const auto pos = rightToLeftGainSlider.getPositionOfValue(s);
        g.drawLine({Point<float>{width / 4, pos}, Point<float>{3 * width / 4, pos}}, 2.f);
    }
}

void MixerComponent::resized() {
    Grid grid;

    using Track = Grid::TrackInfo;
    using Fr = Grid::Fr;

    grid.templateRows = {
        Track(Fr(1))
    };
    grid.templateColumns = {
        Track(Fr(2)),
        Track(Fr(1)),
        Track(Fr(1)),
        Track(Fr(1)),
        Track(Fr(1)),
        Track(Fr(2))
    };

    grid.items = {
        GridItem(),
        GridItem(rightToLeftGainSlider),
        GridItem(leftPreGainSlider),
        GridItem(rightPreGainSlider),
        GridItem(leftToRightGainSlider),
        GridItem()
    };

    grid.performLayout(getLocalBounds());
}
