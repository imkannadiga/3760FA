package com.example.controlserver.Controllers;

import com.example.controlserver.Services.SocketConnectionHandler;

import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.socket.config.annotation.EnableWebSocket;
import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;

// web socket connections is handled 
// by this class
@Configuration
@EnableWebSocket
public class WebSocketConfig
    implements WebSocketConfigurer {

    @Bean
    public SocketConnectionHandler socketConnectionHandler() {
        return new SocketConnectionHandler();
    }

    // Overriding a method which register the socket
    // handlers into a Registry
    @Override
    public void registerWebSocketHandlers(
        WebSocketHandlerRegistry webSocketHandlerRegistry)
    {
        // For adding a Handler we give the Handler class we
        // created before with End point Also we are managing
        // the CORS policy for the handlers so that other
        // domains can also access the socket
        webSocketHandlerRegistry
            .addHandler(socketConnectionHandler(),"")
            .setAllowedOrigins("*");
    }
}