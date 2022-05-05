## このフォルダについて
このフォルダは，Documents\Arduino\librariesなどにコピペすると，上手くビルドが通るはずです（もうちょっと良いやり方を考えたい…（実験用inoから使うのとMain.inoとかからつかうのと両立させたい．「../」））．
### 仕方が無いので
BunBackupなどのバックアップソフトで，1分おきにlibフォルダを丸々バックアップみたいなことをして対応してます

## include pathの設定
.vscode内c_cpp_properties.json
```
"configurations": [
        {
            "name": "Win32",
            "compilerPath": "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.31.31103/bin/Hostx64/x64/cl.exe",
            "compilerArgs": [],
            "intelliSenseMode": "windows-msvc-x64",
            "includePath": [
                "${workspaceFolder}/**",
                "C:\\Program Files (x86)\\Arduino\\libraries",
                "D:\\Documents\\Arduino\\libraries",
                "C:\\Program Files (x86)\\Arduino\\hardware\\arduino\\avr\\libraries"
            ],
            "forcedInclude": [],
            "cStandard": "c17",
            "cppStandard": "c++17",
            "defines": [
                "_DEBUG",
                "UNICODE",
                "_UNICODE"
            ]
        },
```
settings.json（「Ctrlと,」で開ける）内
```
"C_Cpp.default.browse.path": [
        "${workspaceFolder}/",
        "D:\\Documents\\Arduino\\libraries",
        "C:\\Program Files (x86)\\Arduino\\libraries",
        "C:\\Program Files (x86)\\Arduino\\hardware\\arduino\\avr\\libraries"
    ],
    "C_Cpp.default.includePath": [
        "D:\\Documents\\Arduino\\libraries",
        "C:\\Program Files (x86)\\Arduino\\libraries",
        "C:\\Program Files (x86)\\Arduino\\hardware\\arduino\\avr\\libraries"
```
など適宜設定のこと