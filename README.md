# Off Axis 720 / Back Full — 回転軸 3D教材（Phase 0.5）

モーグルの **Cork 720（FIS正式名称: Off Axis 720）** の回転構造を、選手とコーチが直感的に
理解するための Web ベース 3D 教材。研究用シミュレータではなく **教材**。

単一 HTML ＋ Three.js（CDN）。ブラウザストレージは使わない。

**中心メッセージ**
> TAKEOFF までは H（角運動量）を作れる。TAKEOFF 後は H はほぼ変えられない。
> しかし身体の形を変えれば、その H がどのような回転として現れるかは変えられる。

---

## この版（Phase 0.5）で入れたもの

| | 内容 |
|---|---|
| 技セレクタ | **Cork 720**（Off Axis Jumps / 7op）／ **Back Full**（Inverted Jumps / bF）。FIS上は別カテゴリであることを UI に明記 |
| 会場 | Cork = モーグルキッカー（Phase 0 のグラフィックを維持）／ Back Full = **ウォータージャンプ宮城**の簡略再現 |
| 軸の作り方 | **A: contact-dominant / B: aerial-dominant / C: mixed** の3プリセット |
| UI | コーチング言語（後ろ回転の残し方／上半身の抜け方／軸の作り方／身体の形）。物理用語は Advanced に格納 |
| 主表示 | ワールド鉛直軸 Z ／ H ／ 身体長軸 L ／ Tilt の4つだけ |
| Advanced | L/T/AP の身体3軸、ω（root角速度）、仮説パラメータ一覧、検算モード |

### この版で直した技術的欠陥（3点）

1. **ω と H を分離した。** `H0` は離地時に確定する prescribed vector（ワールド固定）として独立に持つ。
   `ω` は姿勢から数値微分して出す別量。両者はコード上で別変数・別概念。飛行中央で 30〜43° ずれる。
2. **Tilt の定義を変えた。** `tilt = asin(L̂·Ĥ)`（H に垂直な平面から身体長軸が外れた量）。
   「H と L のなす角」ではない。純粋なバック宙返りで **Tilt = 0°**。
3. **H を水平にハードコードしていない。** Cork の `h0InclinationDeg` は仮説パラメータ。

---

## 層の分け方

コード（`index.html`）はコメントで 0〜7 に区切ってある。

```
0. CONVENTION        座標規約（固定）
1. HYPOTHESIS        モーグル固有の「仮説」パラメータ  ← 実測が出たらここだけ差し替える
2. MODEL             ModelDefinition（セグメント階層・寸法・質量・COM）
3. VENUE             会場定義（形状プロファイル＋3D構築）
4. MotionGenerator   → Canonical Motion Sequence      ← Phase 2 で差し替わるのはここだけ
5. Analyzer          → Derived Metrics（L, tilt, ωWorld, ωBody）
6. Renderer          three.js（会場・スキーヤー・矢印）
7. UI
```

```
PresetGenerator ─┐
PhysicsSolver  ──┼→ Canonical Motion Sequence → Analyzer → Renderer
MeasuredLoader ──┘
```

データ構造と座標規約は **[SCHEMA.md](SCHEMA.md)** に固定してある。

### 空中姿勢の生成方法

root orientation を自由キーフレームで作っていない。次の幾何変数から生成する：

| 変数 | 意味 |
|---|---|
| `H0` | ワールド固定。離地時に確定 |
| `tilt(t)` | 身体長軸が H に垂直な平面から外れる量 ← **A/B/C はここだけを変える** |
| `phi(t)` | L が H の周囲を回る位相（宙返り位相） |
| `psi(t)` | 身体長軸まわりの姿勢位相（ひねり位相） |

```
bodyY (L軸)  = L̂ = cos(tilt)(cosφ e1 + sinφ e2) + sin(tilt) Ĥ
bodyZ (T軸)  = Ĥ の L 直交成分 を長軸まわりに ψ だけ回したもの   （tilt=0 のとき T̂ = Ĥ）
bodyX (AP軸) = bodyY × bodyZ
```

その上に股関節・脊柱・腕・脚の local quaternion をキーフレームで重ねる。

`ψ̇ ∝ sin(tilt)` としてある（tilt がひねりを生む、という確立した関係）。合計ひねり量が
技の仕様に一致するよう正規化しているので、**A/B/C は合計は同じで、ひねりが立ち上がる
タイミングだけが変わる**。これがコーチに見比べてもらいたい差。

---

## Phase 2 / Phase 3 で差し替える箇所

### Phase 2（離地後を角運動量保存の reduced-order physics に）

差し替えるのは **`generateSequence()` の飛行分岐 1箇所だけ**。

- `index.html` の「4. MotionGenerator」ブロック内、`if (!air) { ... } else { ... }` の `else` 側。
- ここが返すのは `rootPosition` / `rootQuaternion` / `segments`（＝Canonical State）のみ。
- `Analyzer` / `Renderer` / `UI` は **変更不要**。`generator.type` の文字列だけ更新する。
- 必要な追加データ：`MODEL.bodies[]` に **慣性テンソル**（現状 mass と dim のみ）。
  セグメント構成は Pelvis / Torso / Arms / Legs+skis の4〜6剛体に集約できるようにしてある。

### Phase 3（実測データ読み込み）

- `MeasuredLoader` を新設し、同じ Canonical Motion Sequence を返す。
- `takeoff.H0Provenance` を `"measured"` / `"fitted"` に。`H0Confidence`・欠損フラグを追加。
- `frames[].rawObservation` と `frames[].fittedCanonicalState` を分離（SCHEMA.md 6章）。
- **比較は同じ Analyzer を通した指標**（tilt(t), ωL(t), ωT(t), ωAP(t)）で行う。
  シミュレーション専用式と実測専用式を作らない。

---

## 仮説パラメータの置き場所

`index.html` の **`HYPOTHESIS` オブジェクト1箇所**（「1. HYPOTHESIS」ブロック）。
各項目は `{ value, hyp, note }` を持ち、`hyp: true` のものは UI（BODY AXES → Advanced）にも
「［仮説］」と表示される。

| 技 | パラメータ | 既定値 | 仮説か |
|---|---|---|---|
| Cork 720 | `h0InclinationDeg` H0の水平からの傾き | 18° | **仮説** |
| | `somersaultTurns` H まわりの周回 φ | 1.00 | **仮説** |
| | `twistTurns` 長軸まわり ψ | 1.50 | **仮説**（L累積 ≒540° 説に対応） |
| | `tiltMaxDeg` tilt ピーク | 46° | **仮説** |
| Back Full | `h0InclinationDeg` | 0° | 確立寄り（ほぼ純粋な宙返り＋ひねり） |
| | `somersaultTurns` / `twistTurns` | 1.00 / 1.00 | 確立寄り |
| | `tiltMaxDeg` | 24° | **仮説** |

`TILT_PRESETS`（A/B/C の tilt(t) プロファイル）も同じブロックにある。

**確立した原理としてエンジン側にハードコードしてあるもの**（仮説ではない）
- 飛行中 H はほぼ保存される（`H0` は飛行中一定）
- 身体形状（慣性）が変われば同じ H でも回転の速さが変わる（`shapeFactor`）
- ひねりの立ち上がりは tilt が作る（`ψ̇ ∝ sin(tilt)`）
- 着地に向けて tilt を戻すとひねりが止まる（`landFade`）

**数値を確定事実として UI に書かない。** 「Contact 37% / Aerial 63%」のような比率表示は
まだ測定していないので入れていない（Advanced にもその旨を明記）。

---

## 意図的に入れていないもの

- 角度の足し算表示（「L軸540° + T軸180° + AP軸180°」など）
- 結果の角度を原因として直接操作するスライダー（`Cork angle 35°` など）
- H の数値的な大きさ（向きだけを表示）
- contact / aerial の比率の数値
- `Twist input` という名称（「空中でひねりという力が追加される」と読めるため廃止）
- ブラウザストレージ

## 用語

- FIS 正式名称は **Off Axis 720**。「3D」は FIS の正式技名ではない。
- `Rotational Jumps / 720 Position (7p)` と `Off Axis Jumps / Off Axis 720 Position (7op)` は別カテゴリ。
  **Back Full は `Inverted Jumps (bF)` で、さらに別カテゴリ。** コークの一種ではない。
- 「ゼロ角運動量ツイスト（猫ひねり）」「トランポリンのターンテーブル」
  「twisting somersault の tilt 由来ツイスト」を同一視しない。
  ターンテーブルはベッドとの接触で角運動量を受け取れるのでゼロ角運動量運動とは限らない。

---

## 使い方（コーチ向け）

1. 上部で技を選ぶ（Cork 720 / Back Full）。会場・初期値・カメラがまとめて切り替わる。
2. 「台の中」の2つのスライダーで離地条件を作る。
3. ▶ 再生。**離地の瞬間に H 矢印が出て向きが固定され、台の中の入力がロックされる。**
4. 空中の「軸の作り方」を **A と B で見比べる**。合計のひねり量は同じで、
   ひねりが立ち上がるタイミングだけが変わる。実際のコークはどちらに近いか。
5. 「身体の形」は離地後も動かせる（H は同じでも回転の現れ方が変わる）。
6. `BODY AXES` で L/T/AP・ω・仮説パラメータ・検算モードを表示。

`?capture=1` を URL に付けると canvas を画像として保存できる（検証用。描画結果は変わらない）。

---

## 開発メモ

ローカル確認：

```bash
python -m http.server 8000
```

参照素材（`bf.MOV` / `reference/*.jpg`）は選手が写っているため **リポジトリに入れていない**
（`.gitignore` 済み）。会場再現はそれらから起こしてある。
