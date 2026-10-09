/*
    Copyright 2016-2026 melonDS team

    This file is part of melonDS.

    melonDS is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    melonDS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with melonDS. If not, see http://www.gnu.org/licenses/.
*/

#ifndef OPENGLSUPPORT_H
#define OPENGLSUPPORT_H

#include <stdio.h>
#include <string.h>

#include "Platform.h"
#include "PlatformOGL.h"

namespace melonDS::OpenGL
{

void LoadShaderCache();
void SaveShaderCache();

struct AttributeTarget
{
    const char* Name;
    u32 Location;
};


bool CompileVertexFragmentProgram(GLuint& result,
    const std::string& vs, const std::string& fs,
    const std::string& name,
    const std::initializer_list<AttributeTarget>& vertexInAttrs,
    const std::initializer_list<AttributeTarget>& fragmentOutAttrs);

bool CompileComputeProgram(GLuint& result, const std::string& source, const std::string& name);

// debug.litev.glskip (Android, read once): diagnostic GPU pass-skip bitmask, for a per-pass
// GPU cost breakdown (wrong pixels by design). 1 3D polygons, 2 3D edge/fog, 4 2D compositor,
// 8 2D sprites, 16 hi-res final pass, 32 hi-res present copy, 64 hybrid merge draw,
// 128 compositor does not bind the 3D texture, 256 compositor uses a trivial shader,
// 512 no opaque 3D pass, 1024 no translucent 3D passes.
int GLSkip();

// debug.litev.<name> as an int (Android; def when unset or elsewhere)
int Prop(const char* name, int def);

// debug.litev.prof pass counters (any thread; logged by GLRenderer::VBlank every 120 frames)
enum { GLStatComp, GLStatSprites, GLStatFinal, GLStat3D, GLStatFog, GLStatEdge, GLStatShadow, GLStatWBuf, GLStatRunDraws, GLStatN };
void GLStatAdd(int i, int n = 1);
void GLStatLog(int frames);

}

#endif // OPENGLSUPPORT_H
