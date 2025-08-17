package siseon.backend.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.Authentication;
import org.springframework.security.oauth2.server.resource.authentication.JwtAuthenticationToken;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;
import siseon.backend.service.UserService;

@RestController
@RequestMapping("/api/user")
@RequiredArgsConstructor
public class UserController {
    private final UserService userService;

    @DeleteMapping
    public ResponseEntity<Void> withdraw(Authentication auth) {
        String email = ((JwtAuthenticationToken) auth).getToken().getSubject();
        userService.withdrawByEmail(email);
        return ResponseEntity.noContent().build();
    }
}
