1. patch使用Patch_MP_571#####_1.0.371.0_3eddfd8b-a3a229d08d5c6ca40dd76b8cd63ed38f.bin
2. patch externsion使用Patch_ext_MP_master##_1.0.366.0_8adb5306-29c34ebd0e16542d86182d24eaea6153.bin
3. ota header使用OTAHeader_Bank0-07b85be437af61ed4c31cf094cbc4997.bin
4. flash map使用flash map.ini
5. secure boot使用../fsbl_MP_master___1.1.2.0_99b57f16-3e14e3bf53eb7ed34098cbae7dd01680.bin
6. config file使用../OTAHeader_Bank0-a8bab678451b39e424740cd213abd2cf.bin,如果默认已经烧过config file，可以不使用
7. app用工程编译出来的

如果对boot的时间有要求
1. patch使用fast_boot/Patch_MP_master##_1.0.381.0_58fee2ee-bb099aa3f79327bc079089843b99f1be.bin
2. secure boot使用fast_boot/fsbl_MP_master##_1.1.3.0_58fee2ee-252467c0b66a60d43748c129f547ccd7.bin
