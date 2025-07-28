package siseon.backend.service;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.security.oauth2.jose.jws.MacAlgorithm;
import org.springframework.security.oauth2.jwt.*;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.reactive.function.client.WebClient;
import siseon.backend.dto.GoogleUserInfo;
import siseon.backend.dto.JwtResponse;
import siseon.backend.dto.SocialLoginRequest;
import siseon.backend.domain.User;
import siseon.backend.repository.UserRepository;

import java.time.Instant;
import java.time.temporal.ChronoUnit;
import java.util.List;

@Slf4j
@Service
@RequiredArgsConstructor
public class SocialAuthService {

    private final WebClient googleWebClient;
    private final JwtEncoder jwtEncoder;
    private final UserRepository userRepository;

    @Transactional
    public JwtResponse loginWithGoogle(SocialLoginRequest req) {
        // 1) Google UserInfo
        GoogleUserInfo ui = googleWebClient.get()
                .headers(h -> h.setBearerAuth(req.accessToken()))
                .retrieve()
                .bodyToMono(GoogleUserInfo.class)
                .block();

        if (ui == null || ui.email() == null) {
            throw new RuntimeException("Google UserInfo 조회 실패");
        }

        // 2) DB User
        User user = userRepository.findByEmail(ui.email())
                .orElseGet(() -> {
                    String name = ui.name() != null
                            ? ui.name()
                            : ui.email().split("@")[0];
                    User u = User.builder()
                            .email(ui.email())
                            .name(name)
                            .pictureUrl(ui.picture())
                            .build();
                    return userRepository.save(u);
                });

        Instant now = Instant.now();

        // JwsHeader -> HS256
        JwsHeader jwsHeader = JwsHeader.with(MacAlgorithm.HS256).build();

        // 3) Access Token
        JwtClaimsSet accessClaims = JwtClaimsSet.builder()
                .issuer("siseon")
                .issuedAt(now)
                .expiresAt(now.plus(1, ChronoUnit.HOURS))
                .subject(user.getEmail())
                .claim("roles", List.of("USER"))
                .build();
        String accessToken = jwtEncoder.encode(
                JwtEncoderParameters.from(jwsHeader, accessClaims)
        ).getTokenValue();

        // 4) Refresh Token
        JwtClaimsSet refreshClaims = JwtClaimsSet.builder()
                .issuer("siseon")
                .issuedAt(now)
                .expiresAt(now.plus(30, ChronoUnit.DAYS))
                .subject(user.getEmail())
                .build();
        String refreshToken = jwtEncoder.encode(
                JwtEncoderParameters.from(jwsHeader, refreshClaims)
        ).getTokenValue();

        return new JwtResponse(accessToken, refreshToken);
    }
}
