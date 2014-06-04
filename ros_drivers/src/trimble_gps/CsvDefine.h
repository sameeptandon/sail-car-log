////////////////////////////////////////////////////////////////////////////////
//! @file		CsvDefine.h
//! @brief		CsvDefine
//! @details	定義ファイル
//! @date		2013/09/18 Create
//! @author		FSI Hidetaka Kawamura
//
//  Copyright 2013 NISSAN MOTOR CO.,LTD.
////////////////////////////////////////////////////////////////////////////////

#ifndef __CSV_DEFINE_H__
#define __CSV_DEFINE_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


//////////////////////////////////////////////////////////////////////////////
//	define																	//
//////////////////////////////////////////////////////////////////////////////
//------------------
// 共通で使用する値
//------------------
#define CSV_DECIMAL_NUM								(8)							// 小数点以下の桁数
#define CSV_DECIMAL_NUM_DMS_SEC						(4)							// DMSの秒で使用する小数点以下の桁数

//------------------
// 出力ファイル名
//------------------
#define CSV_FILE_NAME_D01							("D01.csv")
#define CSV_FILE_NAME_D02							("D02.csv")
#define CSV_FILE_NAME_D06							("D06.csv")
#define CSV_FILE_NAME_B002							("B002.csv")
#define CSV_FILE_NAME_B003							("B003.csv")
#define CSV_FILE_NAME_M001							("M001.csv")
#define CSV_FILE_NAME_M002							("M002.csv")
#define CSV_FILE_NAME_M003							("M003.csv")
#define CSV_FILE_NAME_R006							("R006.csv")
#define CSV_FILE_NAME_R008							("R008.csv")

//------------------
// 出力データの先頭行の記載内容
//------------------
// D01
#define CSV_DATA_NUM_D01_MCODE						(3)							// メッシュコードの数

#define CSV_DATA_CONTENTS_D01_PID					("PID")						// 識別番号
#define CSV_DATA_CONTENTS_D01_B						("B")						// 位置：緯度
#define CSV_DATA_CONTENTS_D01_L						("L")						// 位置：経度
#define CSV_DATA_CONTENTS_D01_H						("H")						// 位置：高さ
#define CSV_DATA_CONTENTS_D01_BX					("Bx")						// 位置：x
#define CSV_DATA_CONTENTS_D01_LY					("Ly")						// 位置：y
#define CSV_DATA_CONTENTS_D01_REF					("Ref")						// 平面直角座標系
#define CSV_DATA_CONTENTS_D01_MCODE1				("MCODE1")					// 1次メッシュコード
#define CSV_DATA_CONTENTS_D01_MCODE2				("MCODE2")					// 2次メッシュコード
#define CSV_DATA_CONTENTS_D01_MCODE3				("MCODE3")					// 3次メッシュコード
#define CSV_DATA_CONTENTS_D01_GROUND				("Ground")					// 地表点か
#define CSV_DATA_CONTENTS_D01_DATASET				("Dataset")					// 取得時期

#define CSV_DATA_TYPE_D01_PID						("int")						// 識別番号
#define CSV_DATA_TYPE_D01_B							("float")					// 位置：緯度
#define CSV_DATA_TYPE_D01_L							("float")					// 位置：経度
#define CSV_DATA_TYPE_D01_H							("float")					// 位置：高さ
#define CSV_DATA_TYPE_D01_BX						("float")					// 位置：x
#define CSV_DATA_TYPE_D01_LY						("float")					// 位置：y
#define CSV_DATA_TYPE_D01_REF						("int")						// 平面直角座標系
#define CSV_DATA_TYPE_D01_MCODE1					("int")						// 1次メッシュコード
#define CSV_DATA_TYPE_D01_MCODE2					("int")						// 2次メッシュコード
#define CSV_DATA_TYPE_D01_MCODE3					("int")						// 3次メッシュコード
#define CSV_DATA_TYPE_D01_GROUND					("int")						// 地表点か
#define CSV_DATA_TYPE_D01_DATASET					("int")						// 取得時期

// D02
#define CSV_DATA_CONTENTS_D02_LID					("LID")						// 識別番号
#define CSV_DATA_CONTENTS_D02_BPT					("BPT")						// 前ポイントID
#define CSV_DATA_CONTENTS_D02_FPT					("FPT")						// 後ポイントID
#define CSV_DATA_CONTENTS_D02_BLN					("BLN")						// 前ラインID
#define CSV_DATA_CONTENTS_D02_FLN					("FLN")						// 後ラインID
#define CSV_DATA_CONTENTS_D02_PID					("PID")						// 円弧上の1点
#define CSV_DATA_CONTENTS_D02_INVISIBLEFG			("InvisibleFG")				// 陰線FG
#define CSV_DATA_CONTENTS_D02_DATASET				("Dataset")					// 取得時期

#define CSV_DATA_TYPE_D02_LID						("int")						// 識別番号
#define CSV_DATA_TYPE_D02_BPT						("int")						// 前ポイントID
#define CSV_DATA_TYPE_D02_FPT						("int")						// 後ポイントID
#define CSV_DATA_TYPE_D02_BLN						("int")						// 前ラインID
#define CSV_DATA_TYPE_D02_FLN						("int")						// 後ラインID
#define CSV_DATA_TYPE_D02_PID						("int")						// 円弧上の1点
#define CSV_DATA_TYPE_D02_INVISIBLEFG				("int")						// 陰線FG
#define CSV_DATA_TYPE_D02_DATASET					("int")						// 取得時期

// D06
#define CSV_DATA_CONTENTS_D06_AID					("AID")						// 識別番号
#define CSV_DATA_CONTENTS_D06_BLN					("BLN")						// 最初のラインID
#define CSV_DATA_CONTENTS_D06_FLN					("FLN")						// 最後のラインID
#define CSV_DATA_CONTENTS_D06_DATASET				("Dataset")					// 取得時期

#define CSV_DATA_TYPE_D06_AID						("int")						// 識別番号
#define CSV_DATA_TYPE_D06_BLN						("int")						// 最初のラインID
#define CSV_DATA_TYPE_D06_FLN						("int")						// 最後のラインID
#define CSV_DATA_TYPE_D06_DATASET					("int")						// 取得時期

// B002
#define CSV_DATA_CONTENTS_B002_ID					("ID")						// 識別番号
#define CSV_DATA_CONTENTS_B002_PID					("PID")						// ポイントID
#define CSV_DATA_CONTENTS_B002_REFID				("RefID")					// 参照ID
#define CSV_DATA_CONTENTS_B002_KIND					("KIND")					// 種別
#define CSV_DATA_CONTENTS_B002_DATASET				("Dataset")					// 取得時期

#define CSV_DATA_TYPE_B002_ID						("int")						// 識別番号
#define CSV_DATA_TYPE_B002_PID						("int")						// ポイントID
#define CSV_DATA_TYPE_B002_REFID					("int")						// 参照ID
#define CSV_DATA_TYPE_B002_KIND						("int")						// 種別
#define CSV_DATA_TYPE_B002_DATASET					("int")						// 取得時期

// B003
#define CSV_DATA_NUM_B003_LEFTLANE					(2)							// 左車線の数
#define CSV_DATA_NUM_B003_RIGHTLANE					(2)							// 右車線の数
#define CSV_DATA_NUM_B003_OPPOSITELANE				(3)							// 反対車線の数
#define CSV_DATA_NUM_B003_HIGHERPRIORITYLINK		(3)							// HigherPriorityLinkの数
#define CSV_DATA_NUM_B003_LOWERPRIORITYLINK			(3)							// LowerPriorityLinkの数
#define CSV_DATA_NUM_B003_RUNAREA					(4)							// 所属エリアIDの数
#define CSV_DATA_NUM_B003_BID						(3)							// 接続リンクIDの数
#define CSV_DATA_NUM_B003_FID						(3)							// 分岐リンクIDの数

#define CSV_DATA_CONTENTS_B003_ID					("ID")						// 識別番号
#define CSV_DATA_CONTENTS_B003_BID					("BID")						// 手前のリンクID
#define CSV_DATA_CONTENTS_B003_FID					("FID")						// 先のリンクID
#define CSV_DATA_CONTENTS_B003_BNID					("BNID")					// 手前ノードID
#define CSV_DATA_CONTENTS_B003_FNID					("FNID")					// 先ノードID
#define CSV_DATA_CONTENTS_B003_SPEEDMAX				("SpeedMax")				// 制限速度
#define CSV_DATA_CONTENTS_B003_SPEEDAVG				("SpeedAvg")				// 平均速度
#define CSV_DATA_CONTENTS_B003_LENGTH				("Length")					// 距離
#define CSV_DATA_CONTENTS_B003_LANETYPE				("LaneType")				// レーン種類
#define CSV_DATA_CONTENTS_B003_LANECHANGEALLOWED	("LaneChangeAllowed")		// 進路変更
#define CSV_DATA_CONTENTS_B003_LANENUMBER			("LaneNumber")				// レーン番号
#define CSV_DATA_CONTENTS_B003_INSIDEINTERSECTION	("InsideIntersection")		// 交差点内
#define CSV_DATA_CONTENTS_B003_LEFTLANE1			("LeftLane1")				// 同方向：左隣
#define CSV_DATA_CONTENTS_B003_LEFTLANE2			("LeftLane2")				// 同方向：左隣の左隣
#define CSV_DATA_CONTENTS_B003_RIGHTLANE1			("RightLane1")				// 同方向：右隣
#define CSV_DATA_CONTENTS_B003_RIGHTLANE2			("RightLane2")				// 同方向：右隣の右隣
#define CSV_DATA_CONTENTS_B003_OPPOSITELANE1		("OppositeLane1")			// 反対車線：最寄
#define CSV_DATA_CONTENTS_B003_OPPOSITELANE2		("OppositeLane2")			// 反対車線：最寄の外側
#define CSV_DATA_CONTENTS_B003_OPPOSITELANE3		("OppositeLane3")			// 反対車線：外側の外側
#define CSV_DATA_CONTENTS_B003_RUNCTRL				("RunCtrl")					// 走行規制
#define CSV_DATA_CONTENTS_B003_LEFTWIDTH			("LeftWidth")				// 左車線幅
#define CSV_DATA_CONTENTS_B003_LEFTLANEMARKERID		("LeftLaneMarkerID")		// 左白線ID
#define CSV_DATA_CONTENTS_B003_LEFTCURBID			("LeftCurbID")				// 左縁石ID
#define CSV_DATA_CONTENTS_B003_LEFTGUTTERID			("LeftGutterID")			// 左側溝ID
#define CSV_DATA_CONTENTS_B003_LEFTOTHERID			("LeftOtherID")				// 左その他ID
#define CSV_DATA_CONTENTS_B003_RIGHTWIDTH			("RightWidth")				// 右車線幅
#define CSV_DATA_CONTENTS_B003_RIGHTLANEMARKERID	("RightLaneMarkerID")		// 右白線ID
#define CSV_DATA_CONTENTS_B003_RIGHTCURBID			("RightCurbID")				// 右縁石ID
#define CSV_DATA_CONTENTS_B003_RIGHTGUTTERID		("RightGutterID")			// 右側溝ID
#define CSV_DATA_CONTENTS_B003_RIGHTOTHERID			("RightOtherID")			// 右その他ID
#define CSV_DATA_CONTENTS_B003_HIGHERPRIORITYLINK1	("HigherPriorityLink1")		// 優先リンクID
#define CSV_DATA_CONTENTS_B003_HIGHERPRIORITYLINK2	("HigherPriorityLink2")		// 優先リンクID
#define CSV_DATA_CONTENTS_B003_HIGHERPRIORITYLINK3	("HigherPriorityLink3")		// 優先リンクID
#define CSV_DATA_CONTENTS_B003_LOWERPRIORITYLINK1	("LowerPriorityLink1")		// 優先リンクID
#define CSV_DATA_CONTENTS_B003_LOWERPRIORITYLINK2	("LowerPriorityLink2")		// 優先リンクID
#define CSV_DATA_CONTENTS_B003_LOWERPRIORITYLINK3	("LowerPriorityLink3")		// 優先リンクID
#define CSV_DATA_CONTENTS_B003_RUNAREA1				("RunArea1")				// 所属エリアID
#define CSV_DATA_CONTENTS_B003_RUNAREA2				("RunArea2")				// 所属エリアID
#define CSV_DATA_CONTENTS_B003_RUNAREA3				("RunArea3")				// 所属エリアID
#define CSV_DATA_CONTENTS_B003_RUNAREA4				("RunArea4")				// 所属エリアID
#define CSV_DATA_CONTENTS_B003_BID2					("BID2")					// 接続リンクID
#define CSV_DATA_CONTENTS_B003_BID3					("BID3")					// 接続リンクID
#define CSV_DATA_CONTENTS_B003_BID4					("BID4")					// 接続リンクID
#define CSV_DATA_CONTENTS_B003_FID2					("FID2")					// 分岐リンクID
#define CSV_DATA_CONTENTS_B003_FID3					("FID3")					// 分岐リンクID
#define CSV_DATA_CONTENTS_B003_FID4					("FID4")					// 分岐リンクID
#define CSV_DATA_CONTENTS_B003_DATASET				("Dataset")					// 取得時期
#define CSV_DATA_CONTENTS_B003_LID					("LID")						// 車線データID

#define CSV_DATA_TYPE_B003_ID						("int")						// 識別番号
#define CSV_DATA_TYPE_B003_BID						("int")						// 手前のリンクID
#define CSV_DATA_TYPE_B003_FID						("int")						// 先のリンクID
#define CSV_DATA_TYPE_B003_BNID						("int")						// 手前ノードID
#define CSV_DATA_TYPE_B003_FNID						("int")						// 先ノードID
#define CSV_DATA_TYPE_B003_SPEEDMAX					("float")					// 制限速度
#define CSV_DATA_TYPE_B003_SPEEDAVG					("float")					// 平均速度
#define CSV_DATA_TYPE_B003_LENGTH					("float")					// 距離
#define CSV_DATA_TYPE_B003_LANETYPE					("int")						// レーン種類
#define CSV_DATA_TYPE_B003_LANECHANGEALLOWED		("int")						// 進路変更
#define CSV_DATA_TYPE_B003_LANENUMBER				("int")						// レーン番号
#define CSV_DATA_TYPE_B003_INSIDEINTERSECTION		("int")						// 交差点内
#define CSV_DATA_TYPE_B003_LEFTLANE1				("int")						// 同方向：左隣
#define CSV_DATA_TYPE_B003_LEFTLANE2				("int")						// 同方向：左隣の左隣
#define CSV_DATA_TYPE_B003_RIGHTLANE1				("int")						// 同方向：右隣
#define CSV_DATA_TYPE_B003_RIGHTLANE2				("int")						// 同方向：右隣の右隣
#define CSV_DATA_TYPE_B003_OPPOSITELANE1			("int")						// 反対車線：最寄
#define CSV_DATA_TYPE_B003_OPPOSITELANE2			("int")						// 反対車線：最寄の外側
#define CSV_DATA_TYPE_B003_OPPOSITELANE3			("int")						// 反対車線：外側の外側
#define CSV_DATA_TYPE_B003_RUNCTRL					("int")						// 走行規制
#define CSV_DATA_TYPE_B003_LEFTWIDTH				("float")					// 左車線幅
#define CSV_DATA_TYPE_B003_LEFTLANEMARKERID			("int")						// 左白線ID
#define CSV_DATA_TYPE_B003_LEFTCURBID				("int")						// 左縁石ID
#define CSV_DATA_TYPE_B003_LEFTGUTTERID				("int")						// 左側溝ID
#define CSV_DATA_TYPE_B003_LEFTOTHERID				("int")						// 左その他ID
#define CSV_DATA_TYPE_B003_RIGHTWIDTH				("float")					// 右車線幅
#define CSV_DATA_TYPE_B003_RIGHTLANEMARKERID		("int")						// 右白線ID
#define CSV_DATA_TYPE_B003_RIGHTCURBID				("int")						// 右縁石ID
#define CSV_DATA_TYPE_B003_RIGHTGUTTERID			("int")						// 右側溝ID
#define CSV_DATA_TYPE_B003_RIGHTOTHERID				("int")						// 右その他ID
#define CSV_DATA_TYPE_B003_HIGHERPRIORITYLINK1		("int")						// 優先リンクID
#define CSV_DATA_TYPE_B003_HIGHERPRIORITYLINK2		("int")						// 優先リンクID
#define CSV_DATA_TYPE_B003_HIGHERPRIORITYLINK3		("int")						// 優先リンクID
#define CSV_DATA_TYPE_B003_LOWERPRIORITYLINK1		("int")						// 優先リンクID
#define CSV_DATA_TYPE_B003_LOWERPRIORITYLINK2		("int")						// 優先リンクID
#define CSV_DATA_TYPE_B003_LOWERPRIORITYLINK3		("int")						// 優先リンクID
#define CSV_DATA_TYPE_B003_RUNAREA1					("int")						// 所属エリアID
#define CSV_DATA_TYPE_B003_RUNAREA2					("int")						// 所属エリアID
#define CSV_DATA_TYPE_B003_RUNAREA3					("int")						// 所属エリアID
#define CSV_DATA_TYPE_B003_RUNAREA4					("int")						// 所属エリアID
#define CSV_DATA_TYPE_B003_BID2						("int")						// 接続リンクID
#define CSV_DATA_TYPE_B003_BID3						("int")						// 接続リンクID
#define CSV_DATA_TYPE_B003_BID4						("int")						// 接続リンクID
#define CSV_DATA_TYPE_B003_FID2						("int")						// 分岐リンクID
#define CSV_DATA_TYPE_B003_FID3						("int")						// 分岐リンクID
#define CSV_DATA_TYPE_B003_FID4						("int")						// 分岐リンクID
#define CSV_DATA_TYPE_B003_DATASET					("int")						// 取得時期
#define CSV_DATA_TYPE_B003_LID						("int")						// 車線データID

// M001
#define CSV_DATA_NUM_M001_NEARESTLINK				(2)							// NearestLinkの数

#define CSV_DATA_CONTENTS_M001_ID					("ID")						// 識別番号
#define CSV_DATA_CONTENTS_M001_LID					("LID")						// ラインID
#define CSV_DATA_CONTENTS_M001_COLOR				("Color")					// 色
#define CSV_DATA_CONTENTS_M001_LINENUM				("LineNum")					// 線数
#define CSV_DATA_CONTENTS_M001_LINETYPE				("LineType")				// 線種
#define CSV_DATA_CONTENTS_M001_WIDTH				("Width")					// 幅
#define CSV_DATA_CONTENTS_M001_NEARESTLINK1			("NearestLink1")			// もっとも近いリンク
#define CSV_DATA_CONTENTS_M001_NEARESTLINK2			("NearestLink2")			// もっとも近いリンク
#define CSV_DATA_CONTENTS_M001_DATASET				("Dataset")					// 取得時期

#define CSV_DATA_TYPE_M001_ID						("int")						// 識別番号
#define CSV_DATA_TYPE_M001_LID						("int")						// ラインID
#define CSV_DATA_TYPE_M001_COLOR					("char")					// 色
#define CSV_DATA_TYPE_M001_LINENUM					("int")						// 線数
#define CSV_DATA_TYPE_M001_LINETYPE					("int")						// 線種
#define CSV_DATA_TYPE_M001_WIDTH					("float")					// 幅
#define CSV_DATA_TYPE_M001_NEARESTLINK1				("int")						// もっとも近いリンク
#define CSV_DATA_TYPE_M001_NEARESTLINK2				("int")						// もっとも近いリンク
#define CSV_DATA_TYPE_M001_DATASET					("int")						// 取得時期

// M002
#define CSV_DATA_CONTENTS_M002_ID					("ID")						// 識別番号
#define CSV_DATA_CONTENTS_M002_LID					("LID")						// ラインID
#define CSV_DATA_CONTENTS_M002_SIGNALID				("SignalID")				// 対象信号
#define CSV_DATA_CONTENTS_M002_SIGNID				("SignID")					// 対象標識
#define CSV_DATA_CONTENTS_M002_WIDTH				("Width")					// 幅
#define CSV_DATA_CONTENTS_M002_NEARESTLINK			("NearestLink")				// もっとも近いリンク
#define CSV_DATA_CONTENTS_M002_DATASET				("Dataset")					// 取得時期

#define CSV_DATA_TYPE_M002_ID						("int")						// 識別番号
#define CSV_DATA_TYPE_M002_LID						("int")						// ラインID
#define CSV_DATA_TYPE_M002_SIGNALID					("int")						// 対象信号
#define CSV_DATA_TYPE_M002_SIGNID					("int")						// 対象標識
#define CSV_DATA_TYPE_M002_WIDTH					("float")					// 幅
#define CSV_DATA_TYPE_M002_NEARESTLINK				("int")						// もっとも近いリンク
#define CSV_DATA_TYPE_M002_DATASET					("int")						// 取得時期

// M003
#define CSV_DATA_CONTENTS_M003_ID					("ID")						// 識別番号
#define CSV_DATA_CONTENTS_M003_AID					("AID")						// エリアID
#define CSV_DATA_CONTENTS_M003_NEARESTLINK			("NearestLink")				// もっとも近いリンク
#define CSV_DATA_CONTENTS_M003_TYPE					("Type")					// 種別
#define CSV_DATA_CONTENTS_M003_OUTERAID				("OuterAID")				// 相方ID
#define CSV_DATA_CONTENTS_M003_DATASET				("Dataset")					// 取得時期

#define CSV_DATA_TYPE_M003_ID						("int")						// 識別番号
#define CSV_DATA_TYPE_M003_AID						("int")						// エリアID
#define CSV_DATA_TYPE_M003_NEARESTLINK				("int")						// もっとも近いリンク
#define CSV_DATA_TYPE_M003_TYPE						("int")						// 種別
#define CSV_DATA_TYPE_M003_OUTERAID					("int")						// 相方ID
#define CSV_DATA_TYPE_M003_DATASET					("int")						// 取得時期

// R006
#define CSV_DATA_NUM_R006_NEARAREA					(6)							// NearAreaの数

#define CSV_DATA_CONTENTS_R006_ID					("ID")						// 識別番号
#define CSV_DATA_CONTENTS_R006_AID					("AID")						// エリアID
#define CSV_DATA_CONTENTS_R006_NEARESTLINK			("NearestLink")				// もっとも近いリンク
#define CSV_DATA_CONTENTS_R006_NEARAREA1			("NearArea1")				// 隣接エリア
#define CSV_DATA_CONTENTS_R006_NEARAREA2			("NearArea2")				// 隣接エリア
#define CSV_DATA_CONTENTS_R006_NEARAREA3			("NearArea3")				// 隣接エリア
#define CSV_DATA_CONTENTS_R006_NEARAREA4			("NearArea4")				// 隣接エリア
#define CSV_DATA_CONTENTS_R006_NEARAREA5			("NearArea5")				// 隣接エリア
#define CSV_DATA_CONTENTS_R006_NEARAREA6			("NearArea6")				// 隣接エリア
#define CSV_DATA_CONTENTS_R006_TYPE					("Type")					// 種別
#define CSV_DATA_CONTENTS_R006_DATASET				("Dataset")					// 取得時期

#define CSV_DATA_TYPE_R006_ID						("int")						// 識別番号
#define CSV_DATA_TYPE_R006_AID						("int")						// エリアID
#define CSV_DATA_TYPE_R006_NEARESTLINK				("int")						// もっとも近いリンク
#define CSV_DATA_TYPE_R006_NEARAREA1				("int")						// 隣接エリア
#define CSV_DATA_TYPE_R006_NEARAREA2				("int")						// 隣接エリア
#define CSV_DATA_TYPE_R006_NEARAREA3				("int")						// 隣接エリア
#define CSV_DATA_TYPE_R006_NEARAREA4				("int")						// 隣接エリア
#define CSV_DATA_TYPE_R006_NEARAREA5				("int")						// 隣接エリア
#define CSV_DATA_TYPE_R006_NEARAREA6				("int")						// 隣接エリア
#define CSV_DATA_TYPE_R006_TYPE						("int")						// 種別
#define CSV_DATA_TYPE_R006_DATASET					("int")						// 取得時期

// R008
#define CSV_DATA_NUM_R008_SL						(40)						// 所属停止線の数

#define CSV_DATA_CONTENTS_R008_IID					("IID")						// 識別番号
#define CSV_DATA_CONTENTS_R008_TYPE					("TYPE")					// 交差点種別
#define CSV_DATA_CONTENTS_R008_INFOGID				("INFOGID")					// 対象案内板群
#define CSV_DATA_CONTENTS_R008_SL01					("SL01")					// 所属停止線1
#define CSV_DATA_CONTENTS_R008_SL02					("SL02")					// 所属停止線2
#define CSV_DATA_CONTENTS_R008_SL03					("SL03")					// 所属停止線3
#define CSV_DATA_CONTENTS_R008_SL04					("SL04")					// 所属停止線4
#define CSV_DATA_CONTENTS_R008_SL05					("SL05")					// 所属停止線5
#define CSV_DATA_CONTENTS_R008_SL06					("SL06")					// 所属停止線6
#define CSV_DATA_CONTENTS_R008_SL07					("SL07")					// 所属停止線7
#define CSV_DATA_CONTENTS_R008_SL08					("SL08")					// 所属停止線8
#define CSV_DATA_CONTENTS_R008_SL09					("SL09")					// 所属停止線9
#define CSV_DATA_CONTENTS_R008_SL10					("SL10")					// 所属停止線10
#define CSV_DATA_CONTENTS_R008_SL11					("SL11")					// 所属停止線11
#define CSV_DATA_CONTENTS_R008_SL12					("SL12")					// 所属停止線12
#define CSV_DATA_CONTENTS_R008_SL13					("SL13")					// 所属停止線13
#define CSV_DATA_CONTENTS_R008_SL14					("SL14")					// 所属停止線14
#define CSV_DATA_CONTENTS_R008_SL15					("SL15")					// 所属停止線15
#define CSV_DATA_CONTENTS_R008_SL16					("SL16")					// 所属停止線16
#define CSV_DATA_CONTENTS_R008_SL17					("SL17")					// 所属停止線17
#define CSV_DATA_CONTENTS_R008_SL18					("SL18")					// 所属停止線18
#define CSV_DATA_CONTENTS_R008_SL19					("SL19")					// 所属停止線19
#define CSV_DATA_CONTENTS_R008_SL20					("SL20")					// 所属停止線20
#define CSV_DATA_CONTENTS_R008_SL21					("SL21")					// 所属停止線21
#define CSV_DATA_CONTENTS_R008_SL22					("SL22")					// 所属停止線22
#define CSV_DATA_CONTENTS_R008_SL23					("SL23")					// 所属停止線23
#define CSV_DATA_CONTENTS_R008_SL24					("SL24")					// 所属停止線24
#define CSV_DATA_CONTENTS_R008_SL25					("SL25")					// 所属停止線25
#define CSV_DATA_CONTENTS_R008_SL26					("SL26")					// 所属停止線26
#define CSV_DATA_CONTENTS_R008_SL27					("SL27")					// 所属停止線27
#define CSV_DATA_CONTENTS_R008_SL28					("SL28")					// 所属停止線28
#define CSV_DATA_CONTENTS_R008_SL29					("SL29")					// 所属停止線29
#define CSV_DATA_CONTENTS_R008_SL30					("SL30")					// 所属停止線30
#define CSV_DATA_CONTENTS_R008_SL31					("SL31")					// 所属停止線31
#define CSV_DATA_CONTENTS_R008_SL32					("SL32")					// 所属停止線32
#define CSV_DATA_CONTENTS_R008_SL33					("SL33")					// 所属停止線33
#define CSV_DATA_CONTENTS_R008_SL34					("SL34")					// 所属停止線34
#define CSV_DATA_CONTENTS_R008_SL35					("SL35")					// 所属停止線35
#define CSV_DATA_CONTENTS_R008_SL36					("SL36")					// 所属停止線36
#define CSV_DATA_CONTENTS_R008_SL37					("SL37")					// 所属停止線37
#define CSV_DATA_CONTENTS_R008_SL38					("SL38")					// 所属停止線38
#define CSV_DATA_CONTENTS_R008_SL39					("SL39")					// 所属停止線39
#define CSV_DATA_CONTENTS_R008_SL40					("SL40")					// 所属停止線40

#define CSV_DATA_TYPE_R008_IID						("int")						// 識別番号
#define CSV_DATA_TYPE_R008_TYPE						("int")						// 交差点種別
#define CSV_DATA_TYPE_R008_INFOGID					("int")						// 対象案内板群
#define CSV_DATA_TYPE_R008_SL01						("int")						// 所属停止線1
#define CSV_DATA_TYPE_R008_SL02						("int")						// 所属停止線2
#define CSV_DATA_TYPE_R008_SL03						("int")						// 所属停止線3
#define CSV_DATA_TYPE_R008_SL04						("int")						// 所属停止線4
#define CSV_DATA_TYPE_R008_SL05						("int")						// 所属停止線5
#define CSV_DATA_TYPE_R008_SL06						("int")						// 所属停止線6
#define CSV_DATA_TYPE_R008_SL07						("int")						// 所属停止線7
#define CSV_DATA_TYPE_R008_SL08						("int")						// 所属停止線8
#define CSV_DATA_TYPE_R008_SL09						("int")						// 所属停止線9
#define CSV_DATA_TYPE_R008_SL10						("int")						// 所属停止線10
#define CSV_DATA_TYPE_R008_SL11						("int")						// 所属停止線11
#define CSV_DATA_TYPE_R008_SL12						("int")						// 所属停止線12
#define CSV_DATA_TYPE_R008_SL13						("int")						// 所属停止線13
#define CSV_DATA_TYPE_R008_SL14						("int")						// 所属停止線14
#define CSV_DATA_TYPE_R008_SL15						("int")						// 所属停止線15
#define CSV_DATA_TYPE_R008_SL16						("int")						// 所属停止線16
#define CSV_DATA_TYPE_R008_SL17						("int")						// 所属停止線17
#define CSV_DATA_TYPE_R008_SL18						("int")						// 所属停止線18
#define CSV_DATA_TYPE_R008_SL19						("int")						// 所属停止線19
#define CSV_DATA_TYPE_R008_SL20						("int")						// 所属停止線20
#define CSV_DATA_TYPE_R008_SL21						("int")						// 所属停止線21
#define CSV_DATA_TYPE_R008_SL22						("int")						// 所属停止線22
#define CSV_DATA_TYPE_R008_SL23						("int")						// 所属停止線23
#define CSV_DATA_TYPE_R008_SL24						("int")						// 所属停止線24
#define CSV_DATA_TYPE_R008_SL25						("int")						// 所属停止線25
#define CSV_DATA_TYPE_R008_SL26						("int")						// 所属停止線26
#define CSV_DATA_TYPE_R008_SL27						("int")						// 所属停止線27
#define CSV_DATA_TYPE_R008_SL28						("int")						// 所属停止線28
#define CSV_DATA_TYPE_R008_SL29						("int")						// 所属停止線29
#define CSV_DATA_TYPE_R008_SL30						("int")						// 所属停止線30
#define CSV_DATA_TYPE_R008_SL31						("int")						// 所属停止線31
#define CSV_DATA_TYPE_R008_SL32						("int")						// 所属停止線32
#define CSV_DATA_TYPE_R008_SL33						("int")						// 所属停止線33
#define CSV_DATA_TYPE_R008_SL34						("int")						// 所属停止線34
#define CSV_DATA_TYPE_R008_SL35						("int")						// 所属停止線35
#define CSV_DATA_TYPE_R008_SL36						("int")						// 所属停止線36
#define CSV_DATA_TYPE_R008_SL37						("int")						// 所属停止線37
#define CSV_DATA_TYPE_R008_SL38						("int")						// 所属停止線38
#define CSV_DATA_TYPE_R008_SL39						("int")						// 所属停止線39
#define CSV_DATA_TYPE_R008_SL40						("int")						// 所属停止線40

//------------------
// 座標変換計算で使用する値
//------------------
#define HTZ_REF_ID									(9)							// 座標変換で使用する地域のID

// 各地域別の中心緯度経度
#define HTZ_REF_DMS_NUM								(19)								// ダミーを除いたHtzRefのデータ数

#define HTZ_REF_DMS_00								("0.000000000 0.000000000")			// ダミー
#define HTZ_REF_DMS_01								("129.300000000 33.000000000")		//  1 長崎県 鹿児島県のうち北方北緯32度南方北緯27度西方東経128度18分東方東経130度を境界線とする区域内（奄美群島は東経130度13分までを含む。)にあるすべての島、小島、環礁及び岩礁 
#define HTZ_REF_DMS_02								("131.000000000 33.000000000")		//  2 福岡県　佐賀県　熊本県　大分県　宮崎県　鹿児島県（I系に規定する区域を除く。) 
#define HTZ_REF_DMS_03								("132.100000000 36.000000000")		//  3 山口県　島根県　広島県 
#define HTZ_REF_DMS_04								("133.300000000 33.000000000")		//  4 香川県　愛媛県　徳島県　高知県 
#define HTZ_REF_DMS_05								("134.200000000 36.000000000")		//  5 兵庫県　鳥取県　岡山県 
#define HTZ_REF_DMS_06								("136.000000000 36.000000000")		//  6 京都府　大阪府　福井県　滋賀県　三重県　奈良県 和歌山県 
#define HTZ_REF_DMS_07								("137.100000000 36.000000000")		//  7 石川県　富山県　岐阜県　愛知県 
#define HTZ_REF_DMS_08								("138.300000000 36.000000000")		//  8 新潟県　長野県　山梨県　静岡県 
#define HTZ_REF_DMS_09								("139.500000000 36.000000000")		//  9 東京都（XIV系、XVIII系及びXIX系に規定する区域を除く。)　福島県　栃木県　茨城県　埼玉県 千葉県 群馬県　神奈川県 
#define HTZ_REF_DMS_10								("140.500000000 40.000000000")		// 10 青森県　秋田県　山形県　岩手県　宮城県 
#define HTZ_REF_DMS_11								("140.150000000 44.000000000")		// 11 小樽市　函館市　伊達市　北斗市　北海道後志総合振興局の所管区域　北海道胆振総合振興局の所管区域のうち豊浦町、壮瞥町及び洞爺湖町　北海道渡島総合振興局の所管区域　北海道檜山振興局の所管区域
#define HTZ_REF_DMS_12								("142.150000000 44.000000000")		// 12 北海道（XI系及びXIII系に規定する区域を除く。）
#define HTZ_REF_DMS_13								("144.150000000 44.000000000")		// 13 北見市　帯広市　釧路市　網走市　根室市　北海道オホーツク総合振興局の所管区域のうち美幌町、津別町、斜里町、清里町、小清水町、訓子府町、置戸町、佐呂間町及び大空町　北海道十勝総合振興局の所管区域　北海道釧路総合振興局の所管区域　北海道根室振興局の所管区域 
#define HTZ_REF_DMS_14								("142.000000000 26.000000000")		// 14 東京都のうち北緯28度から南であり、かつ東経140度30分から東であり東経143度から西である区域 
#define HTZ_REF_DMS_15								("127.300000000 26.000000000")		// 15 沖縄県のうち東経126度から東であり、かつ東経130度から西である区域 
#define HTZ_REF_DMS_16								("124.000000000 26.000000000")		// 16 沖縄県のうち東経126度から西である区域 
#define HTZ_REF_DMS_17								("131.000000000 26.000000000")		// 17 沖縄県のうち東経130度から東である区域 
#define HTZ_REF_DMS_18								("136.000000000 20.000000000")		// 18 東京都のうち北緯28度から南であり、かつ東経140度30分から西である区域 
#define HTZ_REF_DMS_19								("154.000000000 26.000000000")		// 19 東京都のうち北緯28度から南であり、かつ東経143度から東である区域

//////////////////////////////////////////////////////////////////////////////
//	struct																	//
//////////////////////////////////////////////////////////////////////////////
//------------------
// 入力データ
//------------------

// D01
typedef struct CSV_DATA_VAL_D01
{
	int		m_nPID;									//!< 識別番号
	double	m_dB;									//!< 位置：緯度
	double	m_dL;									//!< 位置：経度
	double	m_dH;									//!< 位置：高さ
	double	m_dBx;									//!< 位置：x
	double	m_dLy;									//!< 位置：ｙ
	int		m_nRef;									//!< 平面直角座標系
	int		m_nMCODE1;								//!< 1次メッシュコード
	int		m_nMCODE2;								//!< 2次メッシュコード
	int		m_nMCODE3;								//!< 3次メッシュコード
	int		m_nGround;								//!< 地表点か
	int		m_nDataset;								//!< 取得時期
}CSV_DATA_VAL_D01;

// D02
typedef struct CSV_DATA_VAL_D02
{
	int		m_nLID;									//!< 識別番号
	int		m_nBPT;									//!< 前ポイントID
	int		m_nFPT;									//!< 後ポイントID
	int		m_nBLN;									//!< 前ラインID
	int		m_nFLN;									//!< 後ラインID
	int		m_nPID;									//!< 円弧上の1点
	int		m_nInvisibleFG;							//!< 陰線FG
	int		m_nDataset;								//!< 取得時期
}CSV_DATA_VAL_D02;

// M002
typedef struct CSV_DATA_VAL_M002
{
	int		m_nID;									//!< 識別番号
	int		m_nLID;									//!< ラインID
	int		m_nSignalID;							//!< 対象信号
	int		m_nSignID;								//!< 対象標識
	double	m_dWidth;								//!< 幅
	int		m_nNearestLink;							//!< もっとも近いリンク
	int		m_nDataset;								//!< 取得時期
}CSV_DATA_VAL_M002;

#endif // __CSV_DEFINE_H__

////////////////////////////////////////////////////////////////////////////////
//	EOF		CsvDefine.h
////////////////////////////////////////////////////////////////////////////////
