# Stereo Vision Toolkit

Changes:
- Show console when running application.
- Add stereo RG video saving/loading for lower sized videos (mono stereo only, previous stereo video type is now called stereo concat).
- Remove doxyfile generated files from repository.
- Fix appcast release info not loading. Release file (release.md) is now referenced as github raw link on prod branch.

Known issues:
- Requires manual restart after installation.
- ROS perception YAML's data arrays must be on single line in YAML file.
- WLS filter causes speckle filter to sometimes be disabled.
- Stereo concat videos are saved uncompressed due to wide video frame size not supported by standard compression codecs. This will create large file sizes. It is advised to use stereo RG format instead however this does not support colour images. 
