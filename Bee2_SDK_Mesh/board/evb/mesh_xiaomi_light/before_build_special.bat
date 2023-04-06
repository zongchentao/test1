@echo off
echo build_before_special bat

bash.exe ..\..\..\tool\version_generation\GitGenerateVersion.sh mesh_app_ ..\..\..\tool\version_generation\ ..\..\..\src\app\mesh\lib\inc\version_app.h