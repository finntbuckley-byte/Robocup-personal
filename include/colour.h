#pragma once
enum ColourSurface { COLOUR_UNKNOWN, COLOUR_FLOOR, COLOUR_GREEN, COLOUR_BLUE };
bool colourInit(); void colourUpdate(); bool colourOk();
ColourSurface colourSurface(); const char* colourName(ColourSurface);
void colourCaptureHome(); ColourSurface colourHome(); bool colourGatingOn();
bool colourOnHomeBase(); bool colourOnEnemyBase(); unsigned long colourOnHomeForMs();
void colourPrintRaw();
