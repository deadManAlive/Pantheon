#pragma once

#include <JuceHeader.h>
#include <memory>

#include "PluginProcessor.h"

#include "LookAndFeel.h"

class FxComponent : public juce::Component
{
public:
    FxComponent(AudioPluginAudioProcessor&, AudioProcessorValueTreeState&);
    ~FxComponent() override;
    void paint(juce::Graphics& g) override;
    void resized() override;
private:
    AudioPluginAudioProcessor& processorRef;
    AudioProcessorValueTreeState& parameters;

    //==============================================================================
    using SliderAttachment = AudioProcessorValueTreeState::SliderAttachment;

    static constexpr int fxPositionRadioButtonId = 777;

    PanLook panLook;

    Label delayLabel;
    Label filterLabel;
    TextButton preFxButton;
    TextButton postFxButton;
    Slider delayLineSlider;
    Slider allPassFreqSlider;

    std::unique_ptr<SliderAttachment> delayLineAttachment;
    std::unique_ptr<SliderAttachment> allPassFreqAttachment;

    GroupComponent border;

    void fxPositionToggleUpdate(bool);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (FxComponent)
};
