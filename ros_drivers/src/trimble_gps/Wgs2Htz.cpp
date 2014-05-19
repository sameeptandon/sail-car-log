////////////////////////////////////////////////////////////////////////////////
//! @file		Wgs2Htz.cpp
//! @brief		Wgs2Htz
//! @details	座標変換クラス
//! @date		2013/09/06 Create
//! @author		FSI Hidetaka Kawamura
//
//  Copyright 2013 NISSAN MOTOR CO.,LTD.
////////////////////////////////////////////////////////////////////////////////

#include "Wgs2Htz.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#define _USE_MATH_DEFINES
#include <math.h>

#include "CsvDefine.h"

#include <cstdlib> // added by Yuta
#include <cmath> // added by Yuta

//////////////////////////////////////////////////////////////////////////////
//	define																	//
//////////////////////////////////////////////////////////////////////////////
#define		D_A				(6378137.0)				// 長半径
#define		D_F				(1.0/298.257222101)		// 扁平率
#define		D_INV_F			(298.257222101)			// 逆扁平率
#define		D_M0			(0.9999)				// 座標系の原点における縮尺係数


//==============================================================================
//! @brief   座標変換クラスインスタンスの実体
//==============================================================================
CWgs2Htz* CWgs2Htz::m_poInstance = NULL;


//==============================================================================
//! @brief			コンストラクタ
//! @return			void
//==============================================================================
CWgs2Htz::CWgs2Htz(void)
//: m_nRefId(HTZ_REF_ID)
//, m_bIsDefinedBasePoint(false)
//, m_oBasePointLon(0.0)
//, m_oBasePointLat(0.0)
//, m_oBasePointLonRad(0.0)
//, m_oBasePointLatRad(0.0)
//, m_dLonPerMeter(0.0)
//, m_dLatPerMeter(0.0)
//, m_dMeterPerLon(0.0)
//, m_dMeterPerLat(0.0)
{
	m_stHtzRefRad.clear();
	// 平面直角座標系の原点テーブルを作成
	//SetHtzTable();
	// 座標変換計算用の固定値を設定
	SetConvertParam();
}

//==============================================================================
//! @brief			デストラクタ
//! @return			void
//==============================================================================
CWgs2Htz::~CWgs2Htz(void)
{
	m_stHtzRefRad.clear();
}

//==============================================================================
//! @brief			座標変換演算関数
//! @param[in]		double					dBreiteDeg			緯度
//! @param[in]		double					dLaengeDeg			経度
//! @param[out]		double&					dOutX				南北方向の距離(m)
//! @param[out]		double&					dOutY				東西方向の距離(m)
//! @return			void
//==============================================================================
void CWgs2Htz::ToHtz(const double dBreiteDeg, const double dLaengeDeg, double& dOutX, double& dOutY) const
{
	ConvertWgs2Htz(dBreiteDeg, dLaengeDeg, m_nRefId, dOutX, dOutY);
}

//==============================================================================
//! @brief			座標変換演算関数（系番号は別のものを使用）
//! @param[in]		double					dBreiteDeg			緯度
//! @param[in]		double					dLaengeDeg			経度
//! @param[in]		double					nRefId				系番号
//! @param[out]		double&					dOutX				南北方向の距離(m)
//! @param[out]		double&					dOutY				東西方向の距離(m)
//! @return			void
//==============================================================================
void CWgs2Htz::ToHtzRefId(const double dBreiteDeg, const double dLaengeDeg, const int nRefId, double& dOutX, double& dOutY) const
{
	ConvertWgs2Htz(dBreiteDeg, dLaengeDeg, nRefId, dOutX, dOutY);
}

//==============================================================================
//! @brief			度数表示から度分秒表示に変換
//! @param[in]		double					dDeg				角度の数値
//! @return			string										角度の文字列（DMS表記）
//==============================================================================
string CWgs2Htz::Deg2Dms(const double dDeg) const
{
	if (-180.0 > dDeg || 180.0 < dDeg)
	{
		// 緯度経度の範囲外の値が入力された場合、空の文字列を返す
		return "";
	}

	ostringstream	oStrRet;								// 戻り値

	int		nDeg			= (int)dDeg;
	double	dMin			= abs((dDeg - nDeg) * 60);
	int		nMin			= (int)dMin;

	double	dSec			= abs((dMin - nMin) * 60);
	string	strSec;
	ostringstream	oStrSec;

	oStrSec << setfill('0') << setw(7) << setiosflags(ios::fixed) << setprecision(CSV_DECIMAL_NUM_DMS_SEC) << dSec;		// 秒の小数点以下4桁まで出力（5桁目は四捨五入される）
	strSec = oStrSec.str();
	strSec.erase(2, 1);											// 3文字目（"."）を削除する

	// 四捨五入により繰り上がりが発生した場合の演算
	if ("600000" == strSec)
	{
		strSec = "000000";
		nMin++;
	}
	if (60 <= nMin)
	{
		nMin = 0;
		if (0 <= dDeg)
		{
			nDeg++;
		}
		else
		{
			nDeg--;
		}
	}

	// 出力の作成
	oStrRet << nDeg << ".";
	oStrRet << setfill('0') << setw(2) << nMin;
	oStrRet << setw(6) << strSec;

	return oStrRet.str();
}

//==============================================================================
//! @brief			度分秒表示から度数表示に変換
//! @param[in]		string					strDms				DMS文字列
//! @return			double										角度の数値
//==============================================================================
double CWgs2Htz::Dms2Deg(const string strDms) const
{
	int		nDeg			= atoi(strDms.substr(0, strDms.find_first_of(".")).c_str());

	string	strAfterPoint	= strDms.substr(strDms.find_first_of(".") + 1);
	int		nMin			= atoi(strAfterPoint.substr(0, 2).c_str());
	string	strSec			= strAfterPoint.substr(2);
	strSec.insert(2, ".");
	double	dSec			= atof(strSec.c_str());

	// 文字列の1文字目が"-"かどうかで出力を変更
	if ("-" == strDms.substr(0, 1))
	{
		return (double)nDeg - ((double)nMin / 60) - (dSec / 3600);
	}
	else
	{
		return (double)nDeg + ((double)nMin / 60) + (dSec / 3600);
	}
}

//==============================================================================
//! @brief			度数表示からラジアン表示に変換
//! @param[in]		double					dDeg				角度の数値
//! @return			double										角度の数値（ラジアン）
//==============================================================================
double CWgs2Htz::Deg2Rad(const double dDeg) const
{
	return M_PI * dDeg / 180.0;
}

//==============================================================================
//! @brief			緯度経度から平面直角座標系に座標変換
//! @param[in]		double					dBreiteDeg			緯度
//! @param[in]		double					dLaengeDeg			経度
//! @param[in]		double					nRefId				系番号
//! @param[out]		double&					dOutX				南北方向の距離(m)
//! @param[out]		double&					dOutY				東西方向の距離(m)
//! @return			void
//==============================================================================
void CWgs2Htz::ConvertWgs2Htz(const double dLatitudeDeg, const double dLongitudeDeg, const int nRefId, double& dOutX, double& dOutY) const
{
	double	psi			= Deg2Rad(dLatitudeDeg);
	double	lambda			= Deg2Rad(dLongitudeDeg);

	//double	psi0			= Deg2Rad(37.410685244096);					
	//double	lambda0			= Deg2Rad(-122.023654674767);				
	double	psi0			= m_oBasePointLatRad;
	double	lambda0			= m_oBasePointLonRad;

	double	Dlambda			= lambda - lambda0;									// 経度-座標系の原点の経度
	double	t				= tan(psi);
	
	
	
	// 計算で求めるパラメータ
	//double	W				= sqrt(1.0-pow(m_dE1,2.0)*pow(sin(psi),2.0));		// （途中計算用）
	double	V				= sqrt(1.0+pow(m_dE2,2.0)*pow(cos(psi),2.0));		// （途中計算用）
	//double	M				= m_dC/(pow(V,3.0));								// 子午線曲率半径
	double	N				= m_dC/V;											// 卯酉線曲率半径
	//double	R				= m_dC/(pow(V,2.0));								// 平均曲率半径
	double	et2				= pow(m_dE2,2.0)*pow(cos(psi),2.0);					//η２乗

	double	S0				= m_dB1*psi0+m_dB2*sin(2*psi0)+m_dB3*sin(4*psi0)+m_dB4*sin(6*psi0)+m_dB5*sin(8*psi0)+m_dB6*sin(10*psi0)+m_dB7*sin(12*psi0)+m_dB8*sin(14*psi0)+m_dB9*sin(16*psi0);
	double	S				= m_dB1*psi+m_dB2*sin(2*psi)+m_dB3*sin(4*psi)+m_dB4*sin(6*psi)+m_dB5*sin(8*psi)+m_dB6*sin(10*psi)+m_dB7*sin(12*psi)+m_dB8*sin(14*psi)+m_dB9*sin(16*psi);


	//緯度経度から平面直角座標x,yを求める計算
	//x座標
	dOutX =
	(
		(S-S0)+N/2.0*pow(cos(psi),2.0)*t*pow(Dlambda,2.0)
		+N/24.0*pow(cos(psi),4.0)*t*(5.0-pow(t,2.0)+9.0*et2+4.0*pow(et2,2.0))*pow(Dlambda,4.0)
		-N/720.0*pow(cos(psi),6.0)*t*(-61.0+58.0*pow(t,2.0)-pow(t,4.0)-270.0*et2+330.0*pow(t,2.0)*et2)*pow(Dlambda,6.0)
		-N/40320.0*pow(cos(psi),8.0)*t*(-1385.0+3111.0*pow(t,2.0)-543.0*pow(t,4.0)+pow(t,6.0))*pow(Dlambda,8.0)
	) * D_M0;

	//y座標
	dOutY = 
	(
		N*cos(psi)*Dlambda
		-N/6.0*pow(cos(psi),3.0)*(-1.0+pow(t,2.0)-et2)*pow(Dlambda,3.0)
		-N/120.0*pow(cos(psi),5.0)*(-5.0+18.0*pow(t,2.0)-pow(t,4.0)-14.0*et2+58.0*pow(t,2.0)*et2)*pow(Dlambda,5.0)
		-N/5040.0*pow(cos(psi),7.0)*(-61.0+479.0*pow(t,2.0)-179.0*pow(t,4.0)+pow(t,6.0))*pow(Dlambda,7.0)
	) * D_M0;
	
}

//==============================================================================
//! @brief			座標変換で使用する値を予め計算しておく関数
//! @return			void
//==============================================================================
void CWgs2Htz::SetConvertParam(void)
{
	// 計算で求めるパラメータ
	//double	b				= D_A*(1.0-D_F);									// 短半径
	m_dC					= D_A/(1.0-D_F);									// 極での極率半径
	m_dE1					= sqrt(2.0*D_F - pow(D_F,2.0));						// 第一離心率
	m_dE2					= sqrt(2.0*D_INV_F - 1.0) / (D_INV_F-1.0);			// 第二離心率
	
	// 途中計算用パラメータ
	double	A				= 1.0+(3.0/4.0)*pow(m_dE1,2.0)+(45.0/64.0)*pow(m_dE1,4.0)+(175.0/256.0)*pow(m_dE1,6.0)+(11025.0/16384.0)*pow(m_dE1,8.0)+(43659.0/65536.0)*pow(m_dE1,10.0)+(693693.0/1048576.0)*pow(m_dE1,12.0)+(19324305.0/29360128.0)*pow(m_dE1,14.0)+(4927697775.0/7516192768.0)*pow(m_dE1,16.0);
	double	B				= (3.0/4.0)*pow(m_dE1,2.0)+(15.0/16.0)*pow(m_dE1,4.0)+(525.0/512.0)*pow(m_dE1,6.0)+(2205.0/2048.0)*pow(m_dE1,8.0)+(72765.0/65536.0)*pow(m_dE1,10.0)+(297297.0/262144.0)*pow(m_dE1,12.0)+(135270135.0/117440512.0)*pow(m_dE1,14.0)+(547521975.0/469762048.0)*pow(m_dE1,16.0);
	double	C				= (15.0/64.0)*pow(m_dE1,4.0)+(105.0/256.0)*pow(m_dE1,6.0)+(2205.0/4096.0)*pow(m_dE1,8.0)+(10395.0/16384.0)*pow(m_dE1,10.0)+(1486485.0/2097152.0)*pow(m_dE1,12.0)+(45090045.0/58720256.0)*pow(m_dE1,14.0)+(766530765.0/939524096.0)*pow(m_dE1,16.0);
	double	D				= (35.0/512.0)*pow(m_dE1,6.0)+(315.0/2048.0)*pow(m_dE1,8.0)+(31185.0/131072.0)*pow(m_dE1,10.0)+(165165.0/524288.0)*pow(m_dE1,12.0)+(45090045.0/117440512.0)*pow(m_dE1,14.0)+(209053845.0/469762048.0)*pow(m_dE1,16.0);
	double	E				= (315.0/16384.0)*pow(m_dE1,8.0)+(3465.0/65536.0)*pow(m_dE1,10.0)+(99099.0/1048576.0)*pow(m_dE1,12.0)+(4099095.0/29360128.0)*pow(m_dE1,14.0)+(348423075.0/1879048192.0)*pow(m_dE1,16.0);
 	double	F				= (693.0/131072.0)*pow(m_dE1,10.0)+(9009.0/524288.0)*pow(m_dE1,12.0)+(4099095.0/117440512.0)*pow(m_dE1,14.0)+(26801775.0/469762048.0)*pow(m_dE1,16.0);
	double	G				= (3003.0/2097152.0)*pow(m_dE1,12.0)+(315315.0/58720256.0)*pow(m_dE1,14.0)+(11486475.0/939524096.0)*pow(m_dE1,16.0);
	double	H				= (45045.0/117440512.0)*pow(m_dE1,14.0)+(765765.0/469762048.0)*pow(m_dE1,16.0);
	double	I				= (765765.0/7516192768.0)*pow(m_dE1,16.0);
	
	m_dB1					= D_A*(1.0-pow(m_dE1,2.0))*A;
	m_dB2					= D_A*(1.0-pow(m_dE1,2.0))*(-B/2.0);
	m_dB3					= D_A*(1.0-pow(m_dE1,2.0))*(C/4.0);
	m_dB4					= D_A*(1.0-pow(m_dE1,2.0))*(-D/6.0);
	m_dB5					= D_A*(1.0-pow(m_dE1,2.0))*(E/8.0);
	m_dB6					= D_A*(1.0-pow(m_dE1,2.0))*(-F/10.0);
	m_dB7					= D_A*(1.0-pow(m_dE1,2.0))*(G/12.0);
	m_dB8					= D_A*(1.0-pow(m_dE1,2.0))*(-H/14.0);
	m_dB9					= D_A*(1.0-pow(m_dE1,2.0))*(I/16.0);
}

//==============================================================================
//! @brief			平面直角座標系で使用する系番号別の原点座標テーブルの作成
//! @return			double
//==============================================================================
void CWgs2Htz::SetHtzTable(void)
{
	SetHtzTablePart(HTZ_REF_DMS_00);
	SetHtzTablePart(HTZ_REF_DMS_01);
	SetHtzTablePart(HTZ_REF_DMS_02);
	SetHtzTablePart(HTZ_REF_DMS_03);
	SetHtzTablePart(HTZ_REF_DMS_04);
	SetHtzTablePart(HTZ_REF_DMS_05);
	SetHtzTablePart(HTZ_REF_DMS_06);
	SetHtzTablePart(HTZ_REF_DMS_07);
	SetHtzTablePart(HTZ_REF_DMS_08);
	SetHtzTablePart(HTZ_REF_DMS_09);
	SetHtzTablePart(HTZ_REF_DMS_10);
	SetHtzTablePart(HTZ_REF_DMS_11);
	SetHtzTablePart(HTZ_REF_DMS_12);
	SetHtzTablePart(HTZ_REF_DMS_13);
	SetHtzTablePart(HTZ_REF_DMS_14);
	SetHtzTablePart(HTZ_REF_DMS_15);
	SetHtzTablePart(HTZ_REF_DMS_16);
	SetHtzTablePart(HTZ_REF_DMS_17);
	SetHtzTablePart(HTZ_REF_DMS_18);
	SetHtzTablePart(HTZ_REF_DMS_19);
}

//==============================================================================
//! @brief			平面直角座標系の原点座標の緯度経度をラジアンの値で設定
//! @param[in]		string					strDmsTxt			"[経度] [緯度]"順で表記されたDMS文字列
//! @return			double
//==============================================================================
void CWgs2Htz::SetHtzTablePart(const string strDmsTxt)
{
	DMS		stDms;
	RAD		stRad;

	stDms.strLongitude = strDmsTxt.substr(0, strDmsTxt.find_first_of(" "));
	stDms.strLatitude = strDmsTxt.substr(strDmsTxt.find_first_of(" ") + 1);

	stRad.dLongitude = Deg2Rad(Dms2Deg(stDms.strLongitude));
	stRad.dLatitude  = Deg2Rad(Dms2Deg(stDms.strLatitude));
	m_stHtzRefRad.push_back(stRad);
}

//==============================================================================
//! @brief			原点座標の設定
//! @param[in]		double					dLongitude			原点の経度
//! @param[in]		double					dLatitude			原点の緯度
//! @return			double
//==============================================================================
void CWgs2Htz::SetBasePoint( const double dLatitude, const double dLongitude)
{
	// 原点の設定
	m_oBasePointLon = dLongitude;
	m_oBasePointLat = dLatitude;
	m_oBasePointLonRad = Deg2Rad(m_oBasePointLon);
	m_oBasePointLatRad = Deg2Rad(m_oBasePointLat);
	m_bIsDefinedBasePoint = true;

	// 原点設定後、移動1m毎の緯度経度の増分を算出
	double dDeltaX;
	double dDeltaY;
	ToHtz(dLatitude + 0.00001, dLongitude + 0.00001, dDeltaX, dDeltaY);
	m_dLonPerMeter = 0.00001 / dDeltaY;
	m_dLatPerMeter = 0.00001 / dDeltaX;
	m_dMeterPerLon = dDeltaY / 0.00001;
	m_dMeterPerLat = dDeltaX / 0.00001;
}

//==============================================================================
//! @brief			原点座標の設定
//! @param[in]		double					dLongitude			原点の経度
//! @param[in]		double					dLatitude			原点の緯度
//! @return			double
//==============================================================================
void CWgs2Htz::SetZeroHeading(const double dLatitude, const double dLongitude )
{
	// 原点の設定
	m_oSecondPointLon = dLongitude;
	m_oSecondPointLat = dLatitude;
	m_oSecondPointLonRad = Deg2Rad(m_oSecondPointLon);
	m_oSecondPointLatRad = Deg2Rad(m_oSecondPointLat);
	m_bIsDefinedSecondPoint = true;

	//find the absolute angle for which the local reference frame is zero
	//point2
	double x2,y2;
	double x,y;
	
	ToHtz(m_oBasePointLat,m_oBasePointLon, x, y);  
	ToHtz(m_oSecondPointLat,m_oSecondPointLon, x2, y2);
    
	m_oBaseLocalZeroHeading = 180/M_PI * atan2(y-y2,x-x2);
    
}
//==============================================================================
//! @brief			東に1m移動した時の経度変化量を取得
//! @return			double
//==============================================================================
double CWgs2Htz::GetLocalZeroHeading(void) const
{
	return m_oBaseLocalZeroHeading;
}
//==============================================================================
//! @brief			東に1m移動した時の経度変化量を取得
//! @return			double
//==============================================================================
double CWgs2Htz::GetLongitudePerMeter(void) const
{
	return m_dLonPerMeter;
}

//==============================================================================
//! @brief			北に1m移動した時の緯度変化量を取得
//! @return			double
//==============================================================================
double CWgs2Htz::GetLatitudePerMeter(void) const
{
	return m_dLatPerMeter;
}

//==============================================================================
//! @brief			東に1度移動した時のメートル変化量を取得
//! @return			double
//==============================================================================
double CWgs2Htz::GetMeterPerLongitude(void) const
{
	return m_dMeterPerLon;
}

//==============================================================================
//! @brief			北に1度移動した時のメートル変化量を取得
//! @return			double
//==============================================================================
double CWgs2Htz::GetMeterPerLatitude(void) const
{
	return m_dMeterPerLat;
}

////////////////////////////////////////////////////////////////////////////////
//	EOF		Wgs2Htz.cpp
////////////////////////////////////////////////////////////////////////////////
