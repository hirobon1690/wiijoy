# wiijoy
## About
Wiiリモコン(RVL-CNT-01, RVL-CNT-01-TR)をROS2のjoyとして機能させるノード．ヌンチャク以外の周辺機器は未検証．
![名称未設定ファイル drawio](https://github.com/hirobon1690/wiijoy/assets/58695125/c2648872-03a2-4da2-b6cc-8f2d6e0f528b)


## Requirements
- [wiiuse](https://github.com/wiiuse/wiiuse)
```bash
git clone https://github.com/wiiuse/wiiuse.git
cd wiiuse
mkdir build
cd build
cmake .. [-DCMAKE_INSTALL_PREFIX=/usr/local] [-DCMAKE_BUILD_TYPE=Release] [-DBUILD_EXAMPLE_SDL=NO]
sudo make install
```

## Usage
### 導入
```bash
cd ~/ros2_ws/src
git clone https://github.com/hirobon1690/wiijoy.git
cd ../
colcon build
source install/setup.bash
```

### 実行
Wiiリモコンの1キーと2キーを同時に押してペアリングモードにしてから
```bash
ros2 run wiijoy joy_node
```

## Configuration
`src/keymap.h`を書き換えてビルドする．
書き方は
```cpp
#define [Wiiのキー] [対応させたいXboxのキー]
```

## Example
https://github.com/hirobon1690/wiijoy/assets/58695125/89d52f40-0dd8-4ca6-b452-d4cea00fdbf8


## License
GNU GPL

## 謝辞
[wiiuse](https://github.com/wiiuse/wiiuse)のExampleを改変させていただきました．
