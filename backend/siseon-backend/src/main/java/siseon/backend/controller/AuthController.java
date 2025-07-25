package siseon.backend.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import siseon.backend.dto.JwtResponse;
import siseon.backend.dto.SocialLoginRequest;
import siseon.backend.service.SocialAuthService;

@RestController
@RequestMapping("/api/auth")
@RequiredArgsConstructor
public class AuthController {

    private final SocialAuthService authService;

    @PostMapping("/google")
    public ResponseEntity<JwtResponse> loginWithGoogle(
            @RequestBody SocialLoginRequest req
    ) {
        JwtResponse jwt = authService.loginWithGoogle(req);
        return ResponseEntity.ok(jwt);
    }
}