package com.example.controlserver.Services;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import com.example.controlserver.Misc.JobStatus;
import com.example.controlserver.Misc.UGVStatus;
import com.example.controlserver.Models.NavigationRequest;
import com.example.controlserver.Models.UGV;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

@Component
public class SocketConnectionHandler extends TextWebSocketHandler {

    private final Map<String, WebSocketSession> webSocketSessions = new ConcurrentHashMap<>();

    private static final Logger logger = LoggerFactory.getLogger(SocketConnectionHandler.class);

    private final ObjectMapper objectMapper = new ObjectMapper();

    @Autowired 
    UGVService ugvService;

    @Autowired
    NavigationRequestService navigationRequestService;

    // This method is executed when client tries to connect
    // to the sockets
    @Override
    public void afterConnectionEstablished(WebSocketSession session)
            throws Exception {

        super.afterConnectionEstablished(session);
        // Logging the connection ID with Connected Message
        // System.out.println(session.getId() + " Connected");

        // Adding the session into the list
        webSocketSessions.put(session.getId(), session);

        session.sendMessage(new TextMessage(session.getId()));
    }

    // When client disconnect from WebSocket then this
    // method is called
    @Override
    public void afterConnectionClosed(WebSocketSession session,
            CloseStatus status) throws Exception {
        super.afterConnectionClosed(session, status);
        // System.out.println(session.getId()+ " DisConnected");

        // Removing the connection info from the list
        webSocketSessions.remove(session.getId());
    }

    @Override
    public void handleMessage(WebSocketSession session, WebSocketMessage<?> message) throws Exception {
        super.handleMessage(session, message);

        // Parse the message payload into a Map
        ObjectMapper objectMapper = new ObjectMapper();
        Map<String, Object> payload = objectMapper.readValue(message.getPayload().toString(), new TypeReference<>() {
        });

        // Extract response_type
        String responseType = (String) payload.get("response_type");

        // Handle message based on response_type
        if (responseType != null) {
            switch (responseType) {
                case "heartbeat":
                    UGV ugv = ugvService.getUGVById((String) payload.get("ugv_id"));
                    ugv.setStatus(UGVStatus.ONLINE);
                    ugvService.updateUGV(ugv.getId(), ugv);
                    logger.debug("Heartbeat recieved from UGV : "+ugv.getId());
                    break;
                
                case "navigation_callback":
                    NavigationRequest navReq = navigationRequestService.getNavigationRequestById((String) payload.get("request_id"));
                    navReq.setJobStatus(JobStatus.COMPLETED);
                    logger.info("Completed navigation job : "+navReq.getId());
                    navigationRequestService.saveNavigationRequest(navReq);
                    break;

                default:
                    logger.error("Invalid or unknown request in websocket :: "+responseType);
                    break;
            }
        }
    }

    public void sendMessageToClient(String sessionId, Map<String, Object> payload) throws Exception {

        WebSocketSession session = webSocketSessions.getOrDefault(sessionId, null);

        if (session == null || !session.isOpen()) {
            // System.out.println("Client has disconnected or does not exist.......");
            return;
        }

        String message = objectMapper.writeValueAsString(payload);

        session.sendMessage(new TextMessage(message));
    }
}
