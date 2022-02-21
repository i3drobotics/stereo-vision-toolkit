# Stereo Vision Toolkit

Changes:
- Auto detect Titanias with naming convention "I3DR_Titania_[serial number]_[l/r]"
- Add custom welding Titania image orientation configuration

Known issues:
- Requires manual restart after installation.
- ROS perception YAML's data arrays must be on single line in YAML file.
- WLS filter causes speckle filter to sometimes be disabled.
- Stereo concat videos are saved uncompressed due to wide video frame size not supported by standard compression codecs. This will create large file sizes. It is advised to use stereo RG format instead however this does not support colour images. 
