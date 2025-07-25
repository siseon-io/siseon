package siseon.backend.config;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.*;
import org.springframework.web.reactive.function.client.WebClient;

@Configuration
public class WebClientConfig {

    @Bean
    public WebClient googleWebClient(
            @Value("${app.oauth2.google.userinfo-uri}") String userinfoUri
    ) {
        return WebClient.builder()
                .defaultHeader("Content-Type", "application/json")
                .baseUrl(userinfoUri)
                .build();
    }
}
