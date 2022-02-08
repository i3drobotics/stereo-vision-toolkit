# Stereo Vision Toolkit

Changes:
- Upgrade release strategy.

Known issues:
- I3DRSGM not availiable due to issue in automatic builds.
- Requires manual restart after installation.
- ROS perception YAML's data arrays must be on single line in YAML file.
- WLS filter causes speckle filter to sometimes be disabled.
- Stereo videos are saved uncompressed to avoid lossy compression which would cause issues when loading stereo videos. This will create large file sizes.
