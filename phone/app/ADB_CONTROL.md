ADB control for robot face videos
================================

Place your emotion videos in `app/src/main/res/raw/` with simple names (lowercase, underscores OK) and an mp4 extension, for example:

- `default_face.mp4` (the default looping expression)
- `angry.mp4`
- `happy.mp4`

The app expects `default_face.mp4` to exist for the default looping face.

Controlling via adb
-------------------

Send an emotion (this will start looping the named video):

adb shell am broadcast -a com.neethu.robotface.SET_EMOTION --es emotion angry

Reset to default looping face:

adb shell am broadcast -a com.neethu.robotface.RESET

Notes
-----
- The emotion string maps to the resource name in `res/raw` (without extension).
- Until you send RESET, the selected emotion video keeps looping.
- The app uses a single VideoView and plays videos from resources.
