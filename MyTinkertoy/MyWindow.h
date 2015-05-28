#ifndef _MYWINDOW_
#define _MYWINDOW_

#include <stdio.h>
#include "dart/gui/SimWindow.h"

class MyWorld;

class MyWindow : public dart::gui::SimWindow {
public:
    MyWindow();
    virtual ~MyWindow();

    // Override these virtual functions defined in yui::Win3D
    virtual void displayTimer(int _val);
    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void click(int button, int state, int x, int y);
    virtual void drag(int x, int y);

    MyWorld* getWorld() {
        return mWorld;
    }

    void setWorld(MyWorld *_world) {
        mWorld = _world;
    }

protected:
    bool mPlaying;
    int mFrame;
      
    MyWorld *mWorld;
};

#endif
