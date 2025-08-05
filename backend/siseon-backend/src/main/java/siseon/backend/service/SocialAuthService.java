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
import siseon.backend.domain.main.User;
import siseon.backend.repository.main.UserRepository;

import java.time.Instant;
import java.time.temporal.ChronoUnit;
import java.util.List;

@Slf4j
@Service
@RequiredArgsConstructor
public class SocialAuthService {

    private final WebClient googleWebClient;
    private final JwtEncoder jwtEncoder;
    private final JwtDecoder jwtDecoder;
    private final UserRepository userRepository;

    @Transactional
    public JwtResponse loginWithGoogle(SocialLoginRequest req) {
        // 1) Google UserInfo 조회
        GoogleUserInfo ui = googleWebClient.get()
                .headers(h -> h.setBearerAuth(req.accessToken()))
                .retrieve()
                .bodyToMono(GoogleUserInfo.class)
                .block();

        if (ui == null || ui.email() == null) {
            throw new RuntimeException("Google UserInfo 조회 실패");
        }

        // 2) DB에 User 저장 또는 조회
        User user = userRepository.findByEmail(ui.email())
                .orElseGet(() -> {
                    String name = ui.name() != null ? ui.name() : ui.email().split("@")[0];
                    User newUser = User.builder()
                            .email(ui.email())
                            .name(name)
                            .pictureUrl(ui.picture())
                            .build();
                    return userRepository.save(newUser);
                });

        // 3) 토큰 생성 및 반환
        return generateTokens(user.getEmail(), List.of("USER"));
    }

    @Transactional
    public JwtResponse refreshToken(String refreshToken) {
        // 1) Refresh Token Decode & Encode
        Jwt decodedToken = jwtDecoder.decode(refreshToken);
        String email = decodedToken.getSubject();

        // 2) DB user
        userRepository.findByEmail(email)
                .orElseThrow(() -> new RuntimeException("사용자가 존재하지 않습니다."));

        // 3) Token Refresh
        List<String> roles = decodedToken.getClaimAsStringList("roles");
        return generateTokens(email, roles);
    }

    private JwtResponse generateTokens(String subject, List<String> roles) {
        Instant now = Instant.now();
        JwsHeader jwsHeader = JwsHeader.with(MacAlgorithm.HS256).build();

        // Access Token
        JwtClaimsSet accessClaims = JwtClaimsSet.builder()
                .issuer("siseon")
                .issuedAt(now)
                .expiresAt(now.plus(2, ChronoUnit.MINUTES))
                .subject(subject)
                .claim("roles", roles)
                .build();
        String accessToken = jwtEncoder.encode(
                JwtEncoderParameters.from(jwsHeader, accessClaims)
        ).getTokenValue();

        // Refresh Token
        JwtClaimsSet refreshClaims = JwtClaimsSet.builder()
                .issuer("siseon")
                .issuedAt(now)
                .expiresAt(now.plus(7, ChronoUnit.DAYS))
                .subject(subject)
                .claim("roles", roles)
                .build();
        String refreshToken = jwtEncoder.encode(
                JwtEncoderParameters.from(jwsHeader, refreshClaims)
        ).getTokenValue();

        return new JwtResponse(accessToken, refreshToken);
    }
}
