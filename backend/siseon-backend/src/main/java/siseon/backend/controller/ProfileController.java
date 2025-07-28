package siseon.backend.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.annotation.AuthenticationPrincipal;
import org.springframework.security.oauth2.jwt.Jwt;
import org.springframework.web.bind.annotation.*;
import siseon.backend.domain.User;
import siseon.backend.dto.ProfileCreateRequest;
import siseon.backend.dto.ProfileResponse;
import siseon.backend.repository.UserRepository;
import siseon.backend.service.ProfileService;

import java.util.List;

@RestController
@RequestMapping("/api/profile")
@RequiredArgsConstructor
public class ProfileController {

    private final ProfileService profileService;
    private final UserRepository userRepository;

    private User getAuthenticatedUser(Jwt jwt) {
        // sub 클레임에 이메일이 들어있으므로, getSubject() 사용
        String email = jwt.getSubject();
        return userRepository.findByEmail(email)
                .orElseThrow(() -> new RuntimeException("User not found"));
    }

    @GetMapping
    public ResponseEntity<List<ProfileResponse>> getProfiles(
            @AuthenticationPrincipal Jwt jwt
    ) {
        User user = getAuthenticatedUser(jwt);
        return ResponseEntity.ok(profileService.getProfiles(user));
    }

    @GetMapping("/{id}")
    public ResponseEntity<ProfileResponse> getProfileById(
            @PathVariable Long id,
            @AuthenticationPrincipal Jwt jwt
    ) {
        User user = getAuthenticatedUser(jwt);
        return ResponseEntity.ok(profileService.getProfileById(id, user));
    }

    @PostMapping
    public ResponseEntity<ProfileResponse> createProfile(
            @RequestBody ProfileCreateRequest request,
            @AuthenticationPrincipal Jwt jwt
    ) {
        User user = getAuthenticatedUser(jwt);
        return ResponseEntity.ok(profileService.createProfile(request, user));
    }

    @PutMapping("/{id}")
    public ResponseEntity<ProfileResponse> updateProfile(
            @PathVariable Long id,
            @RequestBody ProfileCreateRequest request,
            @AuthenticationPrincipal Jwt jwt
    ) {
        User user = getAuthenticatedUser(jwt);
        return ResponseEntity.ok(profileService.updateProfile(id, request, user));
    }

    @DeleteMapping("/{id}")
    public ResponseEntity<Void> deleteProfile(
            @PathVariable Long id,
            @AuthenticationPrincipal Jwt jwt
    ) {
        User user = getAuthenticatedUser(jwt);
        profileService.deleteProfile(id, user);
        return ResponseEntity.noContent().build();
    }
}
