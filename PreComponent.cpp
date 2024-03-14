#include "PreComponent.h"

PreComponent::PreComponent(AudioPluginAudioProcessor& p, AudioProcessorValueTreeState& apvts)
    : processorRef(p)
    , parameters(apvts)
    , panLook(PanLook::Origin::FromMid)
    , volLook(PanLook::Origin::FromMin)
    , inputGainSlider(Slider::RotaryHorizontalVerticalDrag, Slider::NoTextBox)
    , inputPanSlider(Slider::RotaryHorizontalVerticalDrag, Slider::NoTextBox)
{
    inputGainSlider.setLookAndFeel(&volLook);
    addAndMakeVisible(inputGainSlider);
    inputGainAttachment.reset(new SliderAttachment(parameters, "inputGain", inputGainSlider));

    inputPanSlider.setLookAndFeel(&panLook);
    addAndMakeVisible(inputPanSlider);
    inputPanAttachment.reset(new SliderAttachment(parameters, "inputPan", inputPanSlider));

    border.setLookAndFeel(&panLook);
    border.setText("Pre");
    border.setEnabled(false);
    border.setColour(GroupComponent::outlineColourId, Colours::linen);
    addAndMakeVisible(border);
}

PreComponent::~PreComponent() {}

void PreComponent::paint(juce::Graphics& g) {
    g.fillAll (getLookAndFeel().findColour (ResizableWindow::backgroundColourId));
}

void PreComponent::resized() {
    border.setBounds(getLocalBounds().reduced(4));

    Grid grid;

    using Track = Grid::TrackInfo;
    using Fr = Grid::Fr;

    grid.templateRows = {
        Track(Fr(1)),
        Track(Fr(1)),
    };
    grid.templateColumns = {Track(Fr(1))};

    grid.items = {
        GridItem(inputGainSlider),
        GridItem(inputPanSlider),
    };

    const auto width = getLocalBounds().getWidth();
    grid.performLayout(getLocalBounds().reduced(width / 12));
}
