package com.example.valetparking.Helpers;

import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.client.RestTemplate;

@Configuration
public class HttpHelper {
    @Bean
    public RestTemplate restTemplate() {
        return new RestTemplate();
    }
}