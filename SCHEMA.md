# Canonical Motion Sequence — データスキーマと座標規約

`schemaVersion: "0.5.0"`

このファイルは **保存する正データ（Canonical State）** と **計算で出す派生量（Derived Metrics）** の
境界、および座標規約を固定するためのもの。Phase 2（物理ソルバ）／Phase 3（実測読み込み）で
生成元が変わっても、この構造は変えない。

---

## 1. 座標規約（固定・途中で変えない）

| 項目 | 値 |
|---|---|
| 手系 | 右手系 |
| World Up | `+Y` |
| 進行方向 | `+X` |
| 左右 | `+Z`（選手から見て左） |
| 身体長軸 **L** | ローカル `+Y`（頭方向） |
| 前後軸 **AP** | ローカル `+X`（腹側が正） |
| 左右軸 **T**（宙返り軸） | ローカル `+Z`（左が正） |
| バック宙返り | ワールド `+Z` まわりの正回転 |
| quaternion 成分順 | `(x, y, z, w)` |
| 単位 | 長さ m ／ 時間 s ／ 角度 rad ／ 角速度 rad/s |

> **注意**：UI では鉛直基準軸を「Z」と呼んでいる（コーチング上の呼称）。
> 内部座標では鉛直は `+Y`。混同しないこと。

補足：ブーツとスキーは **長辺を X（進行方向）に取る**。Z に長くするとスノーボードになる。

---

## 2. トップレベル構造

```jsonc
{
  "schemaVersion": "0.5.0",
  "coordinateConvention": { /* 上の表と同じ内容をそのまま埋め込む */ },

  "model": { "athlete": { "heightM": 1.72, "massKg": 68 },
             "standingRootHeight": 1.05 },

  "trick": { "key": "cork" | "bf",
             "label": "Cork 720" | "Back Full",
             "fisCategory": "Off Axis Jumps (7op)" | "Inverted Jumps (bF)" },

  "venue": { "key": "cork" | "bf", "name": "モーグルキッカー" | "ウォータージャンプ宮城" },

  "generator": { "type": "H-constrained kinematic scaffold",   // Phase2 では "reduced-order physics"
                 "version": "0.5.0",
                 "note": "物理ソルバではない" },

  "takeoff": {
    "H0": [x, y, z],                 // ★ 離地時に確定する prescribed vector（ワールド固定・向きのみ意味を持つ）
    "H0Provenance": "prescribed",    // Phase3 では "measured" / "fitted"
    "phi0": 0.588,                   // 離地時の宙返り位相 [rad]
    "tiltAtTakeoff": 0.738,          // [rad]
    "tiltMax": 0.803,                // [rad]
    "psiTotal": 9.93, "phiTotal": 5.04,   // [rad]
    "preset": "A" | "B" | "C"
  },

  "events":    { "TAKEOFF": 0, "APEX": 0.637, "LANDING": 1.782 },
  "timeRange": { "start": -1.2, "end": 2.062, "takeoff": 0, "landing": 1.782 },
  "params":    { "tendency": 0.72, "exit": 0.66, "shape": 0.12,
                 "preset": "A", "verify": false },

  "frames": [ /* 下記 */ ]
}
```

---

## 3. frames[] — Canonical State（これだけが正データ）

```jsonc
{
  "t": 0.3444,                       // [s] 離地を 0 とする
  "phase": "APPROACH" | "RAMP" | "FLIGHT" | "LANDING",
  "contactState": "contact" | "aerial",
  "flightPhaseNormalized": 0.193,    // 離地0 → 着地1。contact 中は null
  "rootPosition":   [x, y, z],       // 骨盤原点のワールド位置 [m]
  "rootQuaternion": [x, y, z, w],    // 骨盤のワールド姿勢
  "segments": {                      // 各関節の **ローカル** quaternion（親から見た相対）
    "spine":     [x,y,z,w],
    "neck":      [x,y,z,w],
    "shoulderL": [x,y,z,w], "shoulderR": [x,y,z,w],
    "elbowL":    [x,y,z,w], "elbowR":    [x,y,z,w],
    "hipL":      [x,y,z,w], "hipR":      [x,y,z,w],
    "kneeL":     [x,y,z,w], "kneeR":     [x,y,z,w],
    "ankleL":    [x,y,z,w], "ankleR":    [x,y,z,w]
  }
}
```

**rootQuaternion は符号を連続化してある**（隣接フレームの内積が負なら符号反転）。
ω を数値微分で出すために必須。

---

## 4. Derived Metrics — 正データではない。必ず Analyzer で再計算する

`Analyzer.analyze(sequence)` が frames と同じ長さの配列を返す。**保存しない。**

| キー | 定義 |
|---|---|
| `Lhat`  | `rootQuaternion * (0,1,0)` — 身体長軸 |
| `That`  | `rootQuaternion * (0,0,1)` — 宙返り軸 |
| `APhat` | `rootQuaternion * (1,0,0)` — 前後軸 |
| `Hhat`  | `normalize(takeoff.H0)` — 飛行中は一定 |
| `tilt`  | **`asin(abs(dot(Lhat, Hhat)))`** [rad] |
| `omegaWorld` | `2 * q̇ ⊗ q*` のベクトル部（中心差分） |
| `omegaBody`  | `{ AP, L, T } = rootQuaternion⁻¹ * omegaWorld` |
| `omegaMag`   | `|omegaWorld|` |

### tilt の定義について（重要）

```
tilt = asin(L̂ · Ĥ)
```

「身体長軸が、**H に垂直な平面**からどれだけ外れたか」。

- 純粋なバック宙返り → `L ⟂ H` → **tilt = 0°**
- そこから L が H 方向へ倒れるほど tilt ↑ → **ひねりが立ち上がる**

**「H と L のなす角」ではない。** その定義だと普通のバック宙返りが tilt = 90° と表示され、
教材として逆効果になる（Yeadon 系 twisting-somersault 文献の定義に合わせている）。

第1版では符号を使わず `abs()` を取っている。符号付きにする場合はここだけを変える。

### ω と H を混同しないこと

`H0` は **離地時に確定する prescribed vector**、`omegaWorld` は **姿勢から数値微分した角速度**。
剛体なら `H = Iω` だが、人体＋スキーのように方向によって慣性が大きく異なる系では一般に
`H ∦ ω` になる（本アプリでも飛行中央で 30〜43° ずれる）。多剛体系ではさらに各セグメントの
相対運動による角運動量が加わる。**両者を同じ変数・同じ矢印にしないこと。**

---

## 5. ModelDefinition（フレーム外）

`MODEL` として index.html 内に持つ。Phase 2 で H を計算するのに必要。

- `joints[]` : `{ name, parent, offset[3] }` — 関節中心の階層とオフセット
- `bodies[]` : `{ name, joint, shape, dim[], com[3], mass, color }` — 描画体＋質量＋COMオフセット
- Phase 2 で追加が必要：各 body の **慣性テンソル**（現状は未定義。mass と dim から近似可）

セグメント構成は Phase 2 の reduced-order model（Pelvis / Torso / Arms / Legs+skis の4〜6剛体）
に集約できるよう分けてある。**胸郭（thorax）と骨盤（pelvis）は一体化していない。**

---

## 6. Phase 3（実測）で拡張する箇所

まだ実装していないが、構造上ここに入る：

- `frames[].rawObservation` — 生の観測値（マーカー座標など）
- `frames[].fittedCanonicalState` — 当てはめ後。**raw と processed を混ぜない**
- `takeoff.H0Provenance` を `"measured"` / `"fitted"` にする
- `takeoff.H0Confidence`、`frames[].missing[]` — 欠損フラグ
- 「仮説 vs 実測」の比較は **同じ Analyzer を通した指標**（tilt(t), ωL(t), ωT(t), ωAP(t)）で行う。
  シミュレーション専用式と実測専用式を作らない。
