package siseon.backend.controller;

import siseon.backend.dto.UserProfile;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.Authentication;
import org.springframework.security.oauth2.server.resource.authentication.JwtAuthenticationToken;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;
import siseon.backend.service.UserService;

@RestController
@RequestMapping("/api/user")
@RequiredArgsConstructor
public class UserController {
    private final UserService userService;

    @GetMapping("/profile")
    public ResponseEntity<UserProfile> profile(Authentication auth) {
        String email = ((JwtAuthenticationToken) auth).getToken().getSubject();
        UserProfile dto = userService.getUserProfileByEmail(email);
        return ResponseEntity.ok(dto);
    }
}