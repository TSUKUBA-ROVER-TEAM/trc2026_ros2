# trc2026_ros2
鳥取ローバーチャレンジ2026 ROS2ソフトウェア開発リポジトリ

### 動作環境
- Ubuntu 24.04
- ROS 2 Jazzy

### 開発ルール
- 機能の実装はブランチで行い、プルリクエストを出してmainにマージする
- ノードは単一責任の原則に従い、１ノード１機能にする
- パッケージの主機能は１つに、複数の機能を実装するならパッケージを分ける
- パッケージ名の命名規則はtrc2026_xxx(xxxにはパッケージの機能 ex: interface, msg, bringupなど)のprefixをつける
- executable名の末尾には_nodeをつける
- ノード実装はソース(cpp)とヘッダ(hpp)に分け、rclcpp_componentsを用いて実行ファイルを生成する
- ノードを実装したら、インターフェース(topic・service名、型、内容)とパラメータについてパッケージのdocディレクトリにドキュメントを残す

### 依存関係のセットアップ
```
rosdep update
rosdep install --from-paths . --ignore-src -r -y
```

### ビルド
```
colcon build --symlink-install
```

### テスト
```
source install/setup.bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure
```

### ビルドキャッシュクリア
```
rm -rf build/ install/ log/
```
### システムの起動方法
```
source install/setup.bash
ros2 launch trc2026_bringup trc2026.launch.py
```