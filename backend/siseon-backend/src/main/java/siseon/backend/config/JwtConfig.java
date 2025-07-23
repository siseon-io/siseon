package siseon.backend.config;

import com.nimbusds.jose.JWSAlgorithm;
import com.nimbusds.jose.jwk.KeyUse;
import com.nimbusds.jose.jwk.OctetSequenceKey;
import com.nimbusds.jose.jwk.source.JWKSource;
import com.nimbusds.jose.jwk.JWK;
import com.nimbusds.jose.proc.SecurityContext;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.*;
import org.springframework.security.oauth2.jose.jws.MacAlgorithm;
import org.springframework.security.oauth2.jwt.*;

import javax.crypto.SecretKey;
import javax.crypto.spec.SecretKeySpec;
import java.util.List;

@Configuration
public class JwtConfig {

    @Value("${app.jwt.secret}")
    private String jwtSecret;

    private SecretKey secretKey() {
        byte[] keyBytes = java.util.Base64.getUrlDecoder().decode(jwtSecret);
        return new SecretKeySpec(keyBytes, MacAlgorithm.HS256.getName());
    }

    @Bean
    public JwtDecoder jwtDecoder() {
        SecretKey sk = secretKey();
        return NimbusJwtDecoder
                .withSecretKey(sk)
                .macAlgorithm(MacAlgorithm.HS256)
                .build();
    }

    @Bean
    public JwtEncoder jwtEncoder() {
        SecretKey sk = secretKey();

        JWKSource<SecurityContext> jwkSource = (selector, ctx) -> {
            JWK jwk = new OctetSequenceKey.Builder(sk)
                    .algorithm(JWSAlgorithm.HS256)
                    .keyUse(KeyUse.SIGNATURE)
                    .build();
            return List.of(jwk);
        };

        return new NimbusJwtEncoder(jwkSource);
    }
}
