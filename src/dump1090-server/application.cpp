#include "application.hpp"

#include <myx/filesystem/paths.hpp>

#include <csignal>
#include <geodesic.h>

#include <QCommandLineParser>
#include <QDateTime>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonObject>
#include <QSettings>
#include <QTextStream>

#include <regex>

namespace MF = myx::filesystem;

constexpr float k_KnotToMeterSec     { 0.5144444444F };
constexpr float k_FootMinToMeterSec  { 0.00508F };
constexpr float k_FootToMeter        { 3.280839895F };

Application::Application( int& argc, char** argv ) :
	QCoreApplication( argc, argv )
{
}


Application::~Application()
{
	if ( m_source != nullptr ) { pj_free( m_source ); }
	if ( m_dest != nullptr ) { pj_free( m_dest ); }
}


void Application::parseArguments()
{
	QCommandLineParser parser;

	parser.setApplicationDescription( QStringLiteral( "Description" ) );
	parser.addHelpOption();

	QCommandLineOption outputFilename( { QStringLiteral( "ofile" ), QStringLiteral( "output-file" ) },
	                                   tr( "Output filename" ),
	                                   tr( "filename" ) );
	parser.addOption( outputFilename );

	QCommandLineOption textFilename( { QStringLiteral( "tfile" ), QStringLiteral( "text-file" ) },
	                                 tr( "Text filename" ),
	                                 tr( "filename" ) );
	parser.addOption( textFilename );

	parser.process( *this );

	if ( parser.isSet( outputFilename ) )
	{
		auto o { parser.value( outputFilename ) };

		m_outputFile.setFileName( o );
		if ( !m_outputFile.open( QIODevice::WriteOnly | QIODevice::Truncate ) )
		{
			qCritical() << tr( "Can't open output-file for writing " ) << o;
		}
	}

	if ( parser.isSet( textFilename ) )
	{
		auto o { parser.value( textFilename ) };

		m_textFile.setFileName( o );
		if ( !m_textFile.open( QIODevice::WriteOnly | QIODevice::Truncate ) )
		{
			qCritical() << tr( "Can't open text-file for writing " ) << o;
		}
	}
} // Application::parseArguments


void Application::readSettings()
{
	MF::Paths& paths = MF::Paths::instance();
	paths.init( QStringLiteral( "dump1090-server" ), QStringLiteral( "conf" ) );

	auto* settings = new QSettings( paths.configFilePath(), QSettings::IniFormat );

	m_url       = settings->value( QStringLiteral( "url" ), QStringLiteral( "http://127.0.0.1/data/aircraft.json" ) ).toString();
	m_latitude  = settings->value( QStringLiteral( "latitude" ), 56.881443 ).toDouble();
	m_longitude = settings->value( QStringLiteral( "longitude" ), 35.932736 ).toDouble();
	m_height    = settings->value( QStringLiteral( "height" ), 149.1 ).toDouble();

	settings->sync();
	qDebug() << settings->fileName();
	settings->deleteLater();

	setRadarPosition( m_latitude, m_longitude, m_height );
}


void Application::setRadarPosition( double lat, double lon, double alt )
{
	if ( m_source != nullptr ) { pj_free( m_source ); }
	if ( m_dest != nullptr ) { pj_free( m_dest ); }

	auto dest = QString( "+proj=tmerc +lat_0=%1 +lon_0=%2 +ellps=GRS80 +x_0=0 +y_0=0 +z_0=%3 +k_0=1" ).arg( lat ).arg( lon ).arg( alt );

	m_source = pj_init_plus( "+proj=longlat" );
	m_dest   = pj_init_plus( qPrintable( dest ) );
}


void Application::start()
{
	parseArguments();
	readSettings();

	connect( &m_connectionChecker, &QTimer::timeout, this, &Application::checkServer );
	m_connectionChecker.start( 2000 );

	connect( &m_adsbServer, &QTcpServer::newConnection, this, &Application::newConnectionAdsb );
	connect( &m_updSocket,  static_cast< void ( QUdpSocket::* )( QUdpSocket::SocketError ) >( &QUdpSocket::error ),
	         this, &Application::handleError );

	connect( &m_timerDir, &QTimer::timeout, this, &Application::checkDir );
	connect( &m_watcher,  &QFileSystemWatcher::directoryChanged, this, &Application::readJson );

	connect( &m_manager, &QNetworkAccessManager::finished, this, &Application::readWebJson );
	connect( &m_downloadTimer, &QTimer::timeout, this, &Application::requestDownload );

	m_timerDir.start( 5000 );
	m_downloadTimer.start( 2000 );
} // Application::start


void Application::parsingJson( QJsonDocument aircraftDocument )
{
	auto aircraftObjects  { aircraftDocument.object() };

	double nowTimeDouble { aircraftObjects.value( "now" ).toDouble() };
	double nowInt { 0 };
	double nowFrac { 0 };
	nowFrac = std::modf( nowTimeDouble, &nowInt );
	TimeSpec64 nowTime( static_cast< int64_t >( nowInt ), static_cast< int64_t >( nowFrac * 1e9 ) );
	if ( m_lastNowTime == nowTime )
	{
		m_downloadTimer.start( 400 );
		return;
	}
	m_downloadTimer.start( 2000 );
	m_lastNowTime = nowTime;
	qDebug() << nowTime.seconds();
	for ( const auto& ao: aircraftObjects.value( "aircraft" ).toArray() )
	{
		auto aircraft    = ao.toObject();
		auto missesCount = static_cast< uint8_t >( aircraft.value( "seen" ).toDouble() );
		auto hexIndex    = aircraft.value( "hex" ).toString().toInt( nullptr, 16 );
		auto category    = aircraft.value( "category" ).toString().toLocal8Bit();
		if ( missesCount > 5 )
		{
			if ( m_aircrafts.contains( hexIndex ) )
			{
				auto lastAdsb = m_aircrafts.take( hexIndex );
				lastAdsb.setTrackStatus( veer::types::tracking::TrackStatus::Reset );

				AdsbTracksData adsbTracksData;
				adsbTracksData.header().setSendingTime( TimeSpec64() );
				adsbTracksData.tracks().push_back( lastAdsb );
				writeTracks( adsbTracksData );
			}
			continue;
		}

		AdsbTrack adsbTrack;
		if ( !aircraft.value( "lat" ).isUndefined() ||
		     !aircraft.value( "lon" ).isUndefined() ||
		     !aircraft.value( "alt_baro" ).isUndefined() )
		{
			auto lon = aircraft.value( "lon" ).toDouble() * DEG_TO_RAD;
			auto lat = aircraft.value( "lat" ).toDouble() * DEG_TO_RAD;
			auto alt = aircraft.value( "alt_baro" ).toDouble() / k_FootToMeter;

			if ( ( std::abs( lon ) < 1e-5 ) || ( std::abs( lat ) < 1e-5 ) ) { continue; }
			if ( pj_transform( m_source, m_dest, 1, 1, &lon, &lat, &alt ) != 0 ) { continue; }

			auto c = CartesianPoint3F( static_cast< float >( lat ), static_cast< float >( lon ), static_cast< float >( alt ) );
			adsbTrack.target().setPosition( c );
		}

		if ( ( !aircraft.value( "gs" ).isUndefined() &&
		       !aircraft.value( "track" ).isUndefined() ) ||
		     !aircraft.value( "baro_rate" ).isUndefined() )
		{
			adsbTrack.target().setVelocity(
				CartesianVelocity(
					static_cast< float >( ( aircraft.value( "gs" ).toDouble() * k_KnotToMeterSec ) * std::cos( degreeToRadian( aircraft.value( "track" ).toDouble() ) ) ),
					static_cast< float >( ( aircraft.value( "gs" ).toDouble() * k_KnotToMeterSec ) * std::sin( degreeToRadian( aircraft.value( "track" ).toDouble() ) ) ),
					static_cast< float >( aircraft.value( "baro_rate" ).toDouble() * k_FootMinToMeterSec ) ) );
		}

		adsbTrack.target().setSnr( 0 );
		adsbTrack.target().setType( veer::types::tracking::TargetType::Undefined );
		if ( category.size() == 2 )
		{
			if ( category[0] == 'A' )
			{
				if ( ( category[1] >= '1' ) && ( category[1] <= '6' ) )
				{
					adsbTrack.target().setType( veer::types::tracking::TargetType::Airplane );
				}
				if ( category[1] >= '7' )
				{
					adsbTrack.target().setType( veer::types::tracking::TargetType::Helicopter );
				}
			}

			if ( category.startsWith( "B2" ) )
			{
				adsbTrack.target().setType( veer::types::tracking::TargetType::Aerostat );
			}
		}

		if ( m_aircrafts.contains( hexIndex ) )
		{
			if ( !std::isnan( adsbTrack.target().velocity().x() ) && std::fabs( adsbTrack.target().velocity().x() > 1e-6F ) )
			{
				m_aircrafts[ hexIndex ].target().velocity().setX( adsbTrack.target().velocity().x() );
			}
			if ( !std::isnan( adsbTrack.target().velocity().y() ) && std::fabs( adsbTrack.target().velocity().y() > 1e-6F ) )
			{
				m_aircrafts[ hexIndex ].target().velocity().setY( adsbTrack.target().velocity().y() );
			}
			if ( !std::isnan( adsbTrack.target().velocity().h() ) && std::fabs( adsbTrack.target().velocity().h() > 1e-6F ) )
			{
				m_aircrafts[ hexIndex ].target().velocity().setH( adsbTrack.target().velocity().h() );
			}

			if ( !std::isnan( adsbTrack.target().position().x() ) )
			{
				m_aircrafts[ hexIndex ].target().position().setX( adsbTrack.target().position().x() );
			}
			if ( !std::isnan( adsbTrack.target().position().y() ) )
			{
				m_aircrafts[ hexIndex ].target().position().setY( adsbTrack.target().position().y() );
			}
			if ( !std::isnan( adsbTrack.target().position().h() ) )
			{
				m_aircrafts[ hexIndex ].target().position().setH( adsbTrack.target().position().h() );
			}
			m_aircrafts[ hexIndex ].setFormingTime( nowTime );
			m_aircrafts[ hexIndex ].setMissesCount( missesCount );
			m_aircrafts[ hexIndex ].adsbInfo().setId( static_cast< uint32_t >( hexIndex ) );
			m_aircrafts[ hexIndex ].adsbInfo().setBarHeight( adsbTrack.target().position().h() );
		}
		else
		{
			if ( m_nextNumber >= 512 ) { m_nextNumber = 0; }
			m_nextNumber += 1;

			adsbTrack.setNumber( m_nextNumber );
			adsbTrack.setFormingTime( nowTime );
			adsbTrack.setMissesCount( missesCount );
			adsbTrack.adsbInfo().setId( static_cast< uint32_t >( hexIndex ) );
			adsbTrack.setTrackStatus( veer::types::tracking::TrackStatus::Tracking );
			adsbTrack.setInfoSources( veer::types::tracking::InfoSources::Adsb );
			adsbTrack.adsbInfo().setBarHeight( adsbTrack.target().position().h() );
			adsbTrack.setCaptureTime( nowTime );
			m_aircrafts.insert( hexIndex, std::move( adsbTrack ) );
		}

		auto& t  = m_aircrafts[ hexIndex ].target();
		auto& tv = t.velocity();
		auto& tp = t.position();
		if ( !std::isnan( tv.x() ) &&
		     !std::isnan( tv.y() ) &&
		     !std::isnan( tp.x() ) &&
		     !std::isnan( tp.y() ) &&
		     !std::isnan( tp.h() ) )
		{
			if ( m_textFile.isWritable() )
			{
				QByteArray  ba;
				QTextStream ds( &ba, QIODevice::WriteOnly );
				ds.setRealNumberPrecision( 3 );
				ds.setFieldAlignment( QTextStream::AlignRight );
				ds.setRealNumberNotation( QTextStream::FixedNotation );
				ds << m_aircrafts[ hexIndex ].formingTime().seconds() << " "
				   << m_aircrafts[ hexIndex ].adsbInfo().id() << " "
				   << qSetFieldWidth( 4 ) << m_aircrafts[ hexIndex ].number() << " "
				   << qSetFieldWidth( 8 ) << m_aircrafts[ hexIndex ].adsbInfo().barHeight() << " "
				   << qSetFieldWidth( 10 ) << tp.x() << " "
				   << qSetFieldWidth( 10 ) << tp.y() << " "
				   << qSetFieldWidth( 10 ) << tp.h() << " "
				   << qSetFieldWidth( 8 ) << tv.x() << " "
				   << qSetFieldWidth( 8 ) << tv.y() << " "
				   << qSetFieldWidth( 8 ) << tv.h() << endl;
				m_textFile.write( ba );
			}

			AdsbTracksData adsbTracksData;
			adsbTracksData.header().setSendingTime( TimeSpec64() );
			adsbTracksData.tracks().push_back( m_aircrafts[ hexIndex ] );
			writeTracks( adsbTracksData );
		}
	}

	for ( auto it = m_aircrafts.begin(); it != m_aircrafts.end(); )
	{
		if ( nowTime > it->formingTime() + m_timeout )
		{
			it->setTrackStatus( veer::types::tracking::TrackStatus::Reset );

			AdsbTracksData adsbTracksData;
			adsbTracksData.header().setSendingTime( TimeSpec64() );
			adsbTracksData.tracks().push_back( *it );
			writeTracks( adsbTracksData );
			it = m_aircrafts.erase( it );
		}
		else
		{
			++it;
		}
	}
} // Application::parsingJson


void Application::checkDir()
{
	QFileInfo checkFile( "/run/dump1090-fa" );
	if ( checkFile.exists() )
	{
		m_watcher.addPath( checkFile.absoluteFilePath() );
		readJson( "/run/dump1090-fa" );
	}
}


void Application::checkServer()
{
	if ( !m_adsbServer.isListening() )
	{
		m_adsbServer.listen( QHostAddress::AnyIPv4, veer::common::ports::Adsb );
	}
}


void Application::newConnectionAdsb()
{
	auto* client = m_adsbServer.nextPendingConnection();
	if ( client == nullptr ) { return; }
	connect( client, static_cast< void ( QTcpSocket::* )( QTcpSocket::SocketError ) >( &QTcpSocket::error ),
	         this, &Application::clientError );
	connect( client, &QTcpSocket::disconnected, this, &Application::clientDisconnected );
	m_clientsList.append( client );
}


void Application::clientError()
{
	auto* client = qobject_cast< QTcpSocket* >( sender() );
	if ( client == nullptr ) { return; }
	client->disconnectFromHost();
}


void Application::clientDisconnected()
{
	auto* client = qobject_cast< QTcpSocket* >( sender() );
	if ( client == nullptr ) { return; }
	client->deleteLater();
	m_clientsList.removeAll( client );
}


void Application::requestDownload()
{
	m_manager.get( QNetworkRequest( QUrl( m_url ) ) );
}


void Application::handleError( QAbstractSocket::SocketError error )
{
	Q_UNUSED( error )
	m_updSocket.abort();
	qWarning() << "UDP reader: " << m_updSocket.errorString();
} // Application::handleError


void Application::writeTracks( AdsbTracksData& data )
{
	QByteArray  ba;
	QDataStream ds( &ba, QIODevice::WriteOnly );
	ds << data;

	if ( m_outputFile.isWritable() )
	{
		m_outputFile.write( ba );
	}

	if ( m_updSocket.writeDatagram( ba, QHostAddress( "127.0.0.1" ), veer::common::ports::Adsb ) == -1 )
	{
		qWarning() << "Can't send UDP packet";
	}

	for ( auto& client : m_clientsList )
	{
		if ( client->write( ba ) == -1 )
		{
			qWarning() << "Can't send TCP packet";
		}
	}
} // Application::writeTracks


void Application::readWebJson( QNetworkReply* reply )
{
	if ( reply->error() )
	{
		qDebug() << QString( "Error %1" ).arg( reply->errorString() );
		return;
	}
	auto inputJson = QJsonDocument::fromJson( reply->readAll() );
	parsingJson( inputJson );
} // Application::readWebJson


void Application::readJson( const QString& path )
{
	QFile jsonFile( path + "/aircraft.json" );
	if ( jsonFile.exists() )
	{
		jsonFile.open( QFile::ReadOnly );
		auto aircraftDocument { QJsonDocument().fromJson( jsonFile.readAll() ) };
		parsingJson( aircraftDocument );
	}
	emit directoryChanged( path );
	m_timerDir.start( 5000 );
} // Application::readJson


void Application::handlePosixSignal( int signal )
{
	qDebug() << "Got signal: " << signal;
	if ( signal == SIGHUP )
	{
	}
	else
	{
		QCoreApplication::quit();
	}
} // Application::handlePosixSignal
