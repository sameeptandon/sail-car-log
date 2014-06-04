////////////////////////////////////////////////////////////////////////////////
//! @file		Wgs2Htz.h
//! @brief		Wgs2Htz
//! @details	座標変換計算用クラス
//! @date		2013/09/06 Create
//! @author		FSI Hidetaka Kawamura
//
//  Copyright 2013 NISSAN MOTOR CO.,LTD.
////////////////////////////////////////////////////////////////////////////////

#ifndef __WGS_2_HTZ_H__
#define __WGS_2_HTZ_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <string>
#include <vector>
using namespace std;

//==============================================================================
//! @brief		座標変換計算用クラス
//==============================================================================
class CWgs2Htz
{
//------------------
// struct
//------------------
private:
	//==============================================================================
	//! @brief			DMS表示座標系
	//! @details		DMS表示座標系
	//==============================================================================
	typedef struct _DMS
	{
		string		strLongitude;		// 経度
		string		strLatitude;		// 緯度
	} DMS;

	//==============================================================================
	//! @brief			Rad表示座標系
	//! @details		Rad表示座標系
	//==============================================================================
	typedef struct _RAD
	{
		double		dLongitude;			// 経度
		double		dLatitude;			// 緯度
	} RAD;

//--------------
// method
//--------------
public:
	// インスタンス生成
    static CWgs2Htz* GetInstance()
    {
    	if( m_poInstance == NULL )
		{
    		m_poInstance = new CWgs2Htz();
    	}
    	return m_poInstance;
    }

	// インスタンス削除
	static void DeleteInstance()
	{
		if( m_poInstance != NULL )
		{
			delete m_poInstance;
    		m_poInstance = NULL;
    	}
    	return;
	}

	//-----------------------------------
	// Constructor Destructor
	//-----------------------------------
//private:
public: //changed by Yuta 
	CWgs2Htz(void);
	~CWgs2Htz(void);

    //-----------------------------------
    // operator
    //-----------------------------------

    //-----------------------------------
    // AccessMethod
    //-----------------------------------
public:
	// 緯度経度から平面直角座標系に座標変換
	void ToHtz(const double dBreiteDeg, const double dLaengeDeg, double& dOutX, double& dOutY) const;
	// 緯度経度から平面直角座標系に座標変換（RefIdを別指定）
	void ToHtzRefId(const double dBreiteDeg, const double dLaengeDeg, const int nRefId, double& dOutX, double& dOutY) const;
	// 原点座標の設定
	void SetBasePoint(const double dLatitude, const double dLongitude);
	//setZero heading
	void SetZeroHeading(const double dLatitude,const double dLongitude );
	
	// 東に1m移動した時の経度変化量を取得
	double GetLongitudePerMeter(void) const;
	// 北に1m移動した時の緯度変化量を取得
	double GetLatitudePerMeter(void) const;
	// 東に1度移動した時のメートル変化量を取得
	double GetMeterPerLongitude(void) const;
	// 北に1度移動した時のメートル変化量を取得
	double GetMeterPerLatitude(void) const;
	
	double GetLocalZeroHeading(void) const;
	//-----------------------------------
	// Method
	//-----------------------------------
	// 度数表示から度分秒表示に変更
	string Deg2Dms(const double dDeg) const;
	// 度分秒表示から度数表示に変換
	double Dms2Deg(const string strDms) const;
	// 度数表示からラジアン表示に変換
	double Deg2Rad(const double dDeg) const;
private:
	// 緯度経度から平面直角座標系に座標変換
	void ConvertWgs2Htz(const double dBreiteDeg, const double dLaengeDeg, const int nRefId, double& dOutX, double& dOutY) const;
	// 座標変換で使用する値を予め計算しておく関数
	void SetConvertParam(void);
	// 平面直角座標系で使用する系番号別の原点座標テーブルの作成
	void SetHtzTable(void);
	// 平面直角座標系の原点座標の緯度経度をラジアンの値で設定
	void SetHtzTablePart(const string strDmsTxt);

//--------------
// data
//--------------
private:
    static CWgs2Htz*			m_poInstance;					// 唯一のインスタンス

	int							m_nRefId;						// 平面直角座標系で使用する系番号
	vector<RAD>					m_stHtzRefRad;					// 系番号別の原点座標テーブル（ラジアン表示）

	bool						m_bIsDefinedBasePoint;			// 原点座標を設定済みかどうか
	double						m_oBasePointLon;				// 平面直角座標系の原点の経度
	double						m_oBasePointLat;				// 平面直角座標系の原点の緯度
	double						m_oBasePointLonRad;				// 平面直角座標系の原点の経度（ラジアン表示）
	double						m_oBasePointLatRad;				// 平面直角座標系の原点の緯度（ラジアン表示）
	
	bool						m_bIsDefinedSecondPoint;			// 原点座標を設定済みかどうか
	double						m_oSecondPointLon;				// 平面直角座標系の原点の経度
	double						m_oSecondPointLat;				// 平面直角座標系の原点の緯度
	double						m_oSecondPointLonRad;				// 平面直角座標系の原点の経度（ラジアン表示）
	double						m_oSecondPointLatRad;				// 平面直角座標系の原点の緯度（ラジアン表示）

	double						m_oBaseLocalZeroHeading;			// Local Zero Heading

	double						m_dLonPerMeter;					// y方向1メートル移動毎の経度の増分
	double						m_dLatPerMeter;					// x方向1メートル移動毎の緯度の増分

	double						m_dMeterPerLon;					// 経度1度移動毎のy方向メートルの増分
	double						m_dMeterPerLat;					// 緯度1度移動毎のx方向メートルの増分

	// 以下、座標変換計算用の固定値
	double						m_dC;							// 極での極率半径
	double						m_dE1;							// 第一離心率
	double						m_dE2;							// 第二離心率

	double						m_dB1;							// 座標変換の途中計算用パラメータ
	double						m_dB2;							// 　　〃
	double						m_dB3;							// 　　〃
	double						m_dB4;							// 　　〃
	double						m_dB5;							// 　　〃
	double						m_dB6;							// 　　〃
	double						m_dB7;							// 　　〃
	double						m_dB8;							// 　　〃
	double						m_dB9;							// 　　〃
};

#endif //__WGS_2_HTZ_H__

////////////////////////////////////////////////////////////////////////////////
//	EOF		Wgs2Htz.h
////////////////////////////////////////////////////////////////////////////////
