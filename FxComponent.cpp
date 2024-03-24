#include "FxComponent.h"

FxComponent::FxComponent(AudioPluginAudioProcessor& p, AudioProcessorValueTreeState& apvts)
    : processorRef(p)
    , parameters(apvts)
    , panLook(PanLook::Origin::FromMid)
    , delayLineSlider(Slider::RotaryHorizontalVerticalDrag, Slider::NoTextBox)
    , allPassFreqSlider(Slider::RotaryHorizontalVerticalDrag, Slider::NoTextBox)
{
    if (parameters.getRawParameterValue("fxPosition")->load() > 0.5f) {
        preFxButton.setToggleState(true, sendNotification);
    } else {
        postFxButton.setToggleState(true, sendNotification);
    }

    preFxButton.setLookAndFeel(&panLook);
    preFxButton.setClickingTogglesState(true);
    preFxButton.setButtonText("Pre");
    preFxButton.setConnectedEdges(TextButton::ConnectedOnRight);
    preFxButton.setRadioGroupId(fxPositionRadioButtonId);
    preFxButton.onClick = [this](){fxPositionToggleUpdate(true);};
    addAndMakeVisible(preFxButton);

    postFxButton.setLookAndFeel(&panLook);
    postFxButton.setClickingTogglesState(true);
    postFxButton.setButtonText("Post");
    postFxButton.setConnectedEdges(TextButton::ConnectedOnLeft);
    postFxButton.setRadioGroupId(fxPositionRadioButtonId);
    postFxButton.onClick = [this](){fxPositionToggleUpdate(false);};
    addAndMakeVisible(postFxButton);

    delayLineSlider.setLookAndFeel(&panLook);
    addAndMakeVisible(delayLineSlider);
    delayLineAttachment.reset(new SliderAttachment(parameters, "delayLine", delayLineSlider));

    allPassFreqSlider.setLookAndFeel(&panLook);
    addAndMakeVisible(allPassFreqSlider);
    allPassFreqAttachment.reset(new SliderAttachment(parameters, "allPassFreq", allPassFreqSlider));

    delayLabel.setText("Delay", dontSendNotification);
    delayLabel.setJustificationType(Justification::centred);
    delayLabel.setColour(Label::textColourId, Colours::linen);
    delayLabel.setEnabled(false);
    addAndMakeVisible(delayLabel);
    filterLabel.setText("Phase", dontSendNotification);
    filterLabel.setJustificationType(Justification::centred);
    filterLabel.setColour(Label::textColourId, Colours::linen);
    filterLabel.setEnabled(false);
    addAndMakeVisible(filterLabel);

    border.setLookAndFeel(&panLook);
    border.setText("Fx");
    border.setEnabled(false);
    border.setColour(GroupComponent::outlineColourId, Colours::linen);
    addAndMakeVisible(border);
}

FxComponent::~FxComponent() {}

void FxComponent::paint(juce::Graphics& g) {
    g.fillAll (getLookAndFeel().findColour (ResizableWindow::backgroundColourId));
}

void FxComponent::resized() {
    border.setBounds(getLocalBounds().reduced(4));

    Grid grid;

    using Track = Grid::TrackInfo;
    using Fr = Grid::Fr;

    grid.templateRows = {
        Track(Fr(1)),
        Track(Fr(1)),
        Track(Fr(3)),
        Track(Fr(1)),
    };

    grid.templateColumns = {
        Track(Fr(1)),
        Track(Fr(1)),
    };

    grid.items = {
        GridItem(),
        GridItem(),
        GridItem(preFxButton),
        GridItem(postFxButton),
        GridItem(delayLineSlider),
        GridItem(allPassFreqSlider),
        GridItem(delayLabel),
        GridItem(filterLabel),
    };

    const auto width = getLocalBounds().getWidth();
    grid.performLayout(getLocalBounds().reduced(width / 12));
}

void FxComponent::fxPositionToggleUpdate(bool val) {
    parameters.getRawParameterValue("fxPosition") -> store(val ? 1.f : 0.f);
}