#ifndef ADOBE_AGT_VIDEOPANO_VIDEOPANO_LOG_MESSAGE_H
#define ADOBE_AGT_VIDEOPANO_VIDEOPANO_LOG_MESSAGE_H

#include <cstdarg>
#include <cstdio>
#include <iostream>
#ifdef _WIN32
#include <utility/console_color.h>
#endif

namespace adobe_agt {
namespace videopano {

inline void write_log_message( const char *format, ...  ) {
    static const int buffer_size = 1024;
    char buffer[ buffer_size ];
    va_list args;
    va_start( args, format );
    //vprintf( format, args );
    vsnprintf( buffer, buffer_size, format, args );
    va_end( args );
    std::cout<<buffer;
}

#ifdef _WIN32
inline void write_log_message_red( const char *format, ...  ) {
    static const int buffer_size = 1024;
    char buffer[ buffer_size ];
    va_list args;
    va_start( args, format );
    vsnprintf( buffer, buffer_size, format, args );
    va_end( args );
    adobe_agt::utility::console_attribute_type old_color;
    bool can_set_color = adobe_agt::utility::can_set_console_color( std::cout );
    if ( can_set_color )
        old_color = adobe_agt::utility::set_console_foreground_red( std::cout );
    std::cout<<buffer;
    if ( can_set_color )
        adobe_agt::utility::set_console_color( std::cout, old_color );
}

inline void write_log_message_green( const char *format, ...  ) {
    static const int buffer_size = 1024;
    char buffer[ buffer_size ];
    va_list args;
    va_start( args, format );
    vsnprintf( buffer, buffer_size, format, args );
    va_end( args );
    adobe_agt::utility::console_attribute_type old_color;
    bool can_set_color = adobe_agt::utility::can_set_console_color( std::cout );
    if ( can_set_color )
        old_color = adobe_agt::utility::set_console_foreground_green( std::cout );
    std::cout<<buffer;
    if ( can_set_color )
        adobe_agt::utility::set_console_color( std::cout, old_color );
}
#else
#define write_log_message_red   write_log_message
#define write_log_message_green write_log_message
#endif

class videopano_log {
private: 
    // 0: print iteration 
    // 1: print initial and final 
    // 2: print final message
	// 3: print error only
    int _status; 
public:
    explicit videopano_log( int status ) : _status( status ) {}
    int& status( )       {return _status;}
    int  status( ) const {return _status;}

    // Interfaces for sparse_bundle_adjuster3 and sparse_bundle_adjuster4
    void report_iteration_residual( const char* str ) const {
        if ( _status<1 ) write_log_message( "%s", str );
    }
    template <typename T, typename Iterator>
    void report_iteration_residual( std::size_t iter, T cost, Iterator, std::size_t ) const {
        if ( _status<1 ) 
            write_log_message( "( #%d: %f )", iter, cost );
    }
    void report_initial_residual( const char* str ) const {
        if ( _status<2 ) write_log_message( "%s\n", str );
    }
    template <typename T, typename Iterator>
    void report_initial_residual( T cost, Iterator, std::size_t m ) const {
        if ( _status<2 ) 
            write_log_message( "Initial error ( squared ): %f, ( %d terms, average %f ).\n",
                              cost, m, cost/m );
    }
    void report_final_residual( const char* str ) const {
        if ( _status<2 ) write_log_message( "%s\n", str );
    }
    template <typename T, typename Iterator>
    void report_final_residual( T cost, Iterator, std::size_t m ) const {
        if ( _status<2 ) 
            write_log_message( "Final error ( squared ): %f, ( %d terms, average %f ).\n",
                              cost, m, cost/m );
    }
    void report_final_message( const char *format ) const {
        if ( _status<3 ) {
            /*static const int buffer_size = 1024;
            char buffer[ buffer_size ];
            va_list args;
            va_start( args, format );
            vsnprintf( buffer, buffer_size, format, args );
            va_end( args );*/
            write_log_message( "%s\n", format );  
        }
    }
    void report_error( const char* str ) const {
        write_log_message_red( "%s", str );
    }
};

} // namespace videopano
} // namespace adobe_agt

#endif
