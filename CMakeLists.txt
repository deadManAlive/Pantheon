juce_add_plugin(Pantheon
  COMPANY_NAME "Tauri Invictus"
  BUNDLE_ID "com.tauriinvictus.pantheon"
  IS_SYNTH FALSE
  IS_MIDI_EFFECT FALSE
  COPY_PLUGIN_AFTER_BUILD TRUE
  VST3_COPY_DIR "D:/null_/DSP/JUCE/VSTVault"
  PLUGIN_MANUFACTURER_CODE "De4d"
  PLUGIN_CODE "Pth3"
  FORMATS VST3 Standalone
  PRODUCT_NAME "Pantheon Stereo Shaper"
  VST3_CATEGORIES Fx
)

juce_generate_juce_header(Pantheon)

target_sources(Pantheon
  PRIVATE
    FxComponent.cpp
    LookAndFeel.cpp
    MixerComponent.cpp
    PluginEditor.cpp
    PluginProcessor.cpp
    PreComponent.cpp
    Processors.cpp
)

target_compile_definitions(Pantheon
    PUBLIC
        JUCE_WEB_BROWSER=0
        JUCE_USE_CURL=0
        JUCE_VST3_CAN_REPLACE_VST2=0
        JUCE_DISPLAY_SPLASH_SCREEN=0
)

target_link_libraries(Pantheon
    PRIVATE
        juce::juce_audio_utils
        juce::juce_dsp
    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags
)