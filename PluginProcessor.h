#pragma once

#include <JuceHeader.h>
#include <memory>

//==============================================================================
class AudioPluginAudioProcessor  : public juce::AudioProcessor
{
public:
    //==============================================================================
    AudioPluginAudioProcessor();
    ~AudioPluginAudioProcessor() override;

    //==============================================================================
    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;

    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;

    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;
    using AudioProcessor::processBlock;

    //==============================================================================
    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    //==============================================================================
    const juce::String getName() const override;

    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;
    double getTailLengthSeconds() const override;

    //==============================================================================
    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram (int index) override;
    const juce::String getProgramName (int index) override;
    void changeProgramName (int index, const juce::String& newName) override;

    //==============================================================================
    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    //==============================================================================
    AudioProcessorValueTreeState::ParameterLayout createParameterLayout();

private:
    //==============================================================================
    AudioProcessorValueTreeState apvts;

    //==============================================================================
    using IOProcessor = AudioProcessorGraph::AudioGraphIOProcessor;
    using Node = AudioProcessorGraph::Node;

    std::unique_ptr<AudioProcessorGraph> mainProcessor;

    Node::Ptr audioInputNode;
    Node::Ptr preProcessorNode;

    Node::Ptr leftPreGainUnitNode;
    Node::Ptr leftToRightGainUnitNode;
    Node::Ptr rightToLeftGainUnitNode;
    Node::Ptr rightPreGainUnitNode;

    Node::Ptr leftPostProcessorNode;
    Node::Ptr rightPostProcessorNode;
    Node::Ptr audioOutputNode;

    //==============================================================================
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (AudioPluginAudioProcessor)
};