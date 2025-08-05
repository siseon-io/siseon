package siseon.backend.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.annotation.AuthenticationPrincipal;
import org.springframework.security.oauth2.jwt.Jwt;
import org.springframework.web.bind.annotation.*;
import siseon.backend.domain.main.User;
import siseon.backend.dto.ProfileResponse;
import siseon.backend.service.ProfileService;
import siseon.backend.service.PushNotificationService;
import siseon.backend.repository.main.UserRepository;

@RestController
@RequestMapping("/api/push")
@RequiredArgsConstructor
public class PushController {

    private final ProfileService profileService;
    private final PushNotificationService pushService;
    private final UserRepository userRepository;

    private User getUser(Jwt jwt) {
        return userRepository.findByEmail(jwt.getSubject())
                .orElseThrow(() -> new RuntimeException("User not found"));
    }

    @PostMapping("/register")
    public ResponseEntity<Void> registerToken(
            @AuthenticationPrincipal Jwt jwt,
            @RequestParam Long profileId,
            @RequestParam String fcmToken) {

        User me = getUser(jwt);
        profileService.updateFcmToken(profileId, fcmToken, me);
        return ResponseEntity.ok().build();
    }

    @PostMapping("/send-test")
    public ResponseEntity<ProfileResponse> sendTest(
            @AuthenticationPrincipal Jwt jwt,
            @RequestParam Long profileId) {

        User me = getUser(jwt);
        ProfileResponse dto = profileService.getProfileById(profileId, me);

        pushService.sendPushAsync(
                dto.getFcmToken(),
                "테스트 알림",
                "비동기 FCM 전송이 동작합니다."
        );

        return ResponseEntity.ok(dto);
    }
}