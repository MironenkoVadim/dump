#ifndef APPLICATION_HPP_
#define APPLICATION_HPP_

#pragma once

#include <veer/common/ports.hpp>
#include <veer/protocols/tracking/adsb_tracks_packet.hpp>

#include <boost/optional.hpp>

#include <proj_api.h>

#include <QJsonDocument>
#include <QCoreApplication>
#include <QFile>
#include <QFileSystemWatcher>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include <QUdpSocket>
#include <QNetworkReply>
#include <QNetworkAccessManager>

inline double degreeToRadian( const double angle )
{
	constexpr double toRadian = M_PI / 180.0;
	return( angle * toRadian );
}


inline double radianToDegree( const double angle )
{
	constexpr double toDegree = 180.0 / M_PI;
	return( angle * toDegree );
}


class Application : public QCoreApplication
{
	Q_OBJECT

	using CartesianPoint3F  = veer::types::common::CartesianPoint3F;
	using CartesianVelocity = veer::types::common::CartesianVelocity;
	using TimeSpec64        = veer::types::common::TimeSpec64;
	using AdsbTrack         = veer::protocols::tracking::AdsbTrack;
	using AdsbTracksData    = veer::protocols::tracking::AdsbTracksData;

public:
	Application( int& argc, char** argv );
	~Application() final;
	void handlePosixSignal( int signal );
	Q_SLOT void start();

private:
	int                    m_timeout { 10 };
	int                    m_nextNumber { 0 };
	QMap< int, AdsbTrack > m_aircrafts;
	QFile                  m_outputFile;
	QFile                  m_textFile;
	TimeSpec64             m_lastNowTime { 1, 1 };

	QTimer m_timerDir;
	QTimer m_connectionChecker;
	QTimer m_downloadTimer;

	QTcpServer            m_adsbServer;
	QList< QTcpSocket* >  m_clientsList;
	QFileSystemWatcher    m_watcher;
	QUdpSocket            m_updSocket;
	QNetworkAccessManager m_manager;
	QString               m_url;

	double m_longitude { 0.0 };
	double m_latitude { 0.0 };
	double m_height { 0.0 };

	projPJ m_source { nullptr };
	projPJ m_dest { nullptr };
	void parseArguments();
	void readSettings();
	void parsingJson( QJsonDocument );
	void writeTracks( AdsbTracksData& data );
	void setRadarPosition( double lat, double lon, double alt );

	Q_SIGNAL void directoryChanged( const QString& path );

	Q_SLOT void readJson( const QString& path );
	Q_SLOT void handleError( QAbstractSocket::SocketError error );
	Q_SLOT void readWebJson( QNetworkReply* );
	Q_SLOT void checkDir();
	Q_SLOT void checkServer();
	Q_SLOT void newConnectionAdsb();
	Q_SLOT void clientError();
	Q_SLOT void clientDisconnected();
	Q_SLOT void requestDownload();
}; // class Application

#endif // ifndef APPLICATION_HPP_
