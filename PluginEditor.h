#pragma once

#include "PluginProcessor.h"

#include "FxComponent.h"
#include "MixerComponent.h"
#include "PreComponent.h"
#include "PostComponent.h"
// #include "BinaryData.h"

//==============================================================================
class FillerComp : public Component {
public:
    void paint(Graphics& g) override;
};

//==============================================================================
class AudioPluginAudioProcessorEditor  : public juce::AudioProcessorEditor
{
public:
    explicit AudioPluginAudioProcessorEditor (AudioPluginAudioProcessor&, AudioProcessorValueTreeState&);
    ~AudioPluginAudioProcessorEditor() override;

    //==============================================================================
    void paint (juce::Graphics&) override;
    void resized() override;

private:
    using SliderAttachment = AudioProcessorValueTreeState::SliderAttachment;

    // This reference is provided as a quick way for your editor to
    // access the processor object that created it.
    // AudioPluginAudioProcessor& processorRef;
    // AudioProcessorValueTreeState& parameters;
    
    //==============================================================================
    MixerComponent mixerComponent;
    PreComponent preComponent;
    PostComponent postComponent;
    FillerComp filler;
    FxComponent fxComponent;

    //==============================================================================
    PanLook panLook;
    
    //==============================================================================
    GroupComponent border;

    // juce::Image backGroundImg = juce::ImageCache::getFromMemory(BinaryData::bgscribble_png, BinaryData::bgscribble_pngSize);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (AudioPluginAudioProcessorEditor)
};