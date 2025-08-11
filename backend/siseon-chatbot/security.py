from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from jose import jwt
from jose.exceptions import ExpiredSignatureError, JWTClaimsError, JWKError, JOSEError
from pydantic import BaseModel
from typing import List, Optional

from settings import JWT_SECRET_KEY_BYTES, JWT_ALGORITHM, JWT_ISSUER

bearer_scheme = HTTPBearer(auto_error=False)

class TokenData(BaseModel):
    sub: str
    roles: List[str] = []

async def get_current_user(
    creds: Optional[HTTPAuthorizationCredentials] = Depends(bearer_scheme)
) -> TokenData:
    if creds is None or creds.scheme.lower() != "bearer":
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing Bearer token",
            headers={"WWW-Authenticate": "Bearer"},
        )

    token = creds.credentials
    try:
        payload = jwt.decode(
            token,
            JWT_SECRET_KEY_BYTES,          # base64url 디코딩된 바이트 키 (스프링과 동일)
            algorithms=[JWT_ALGORITHM],    # HS256
            issuer=JWT_ISSUER,             # iss = "siseon"
            options={
                "verify_aud": False,       # aud 미검증
                "require_exp": True,       # exp 필수
                "require_iat": True,       # iat 필수
            },
        )
    except ExpiredSignatureError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token expired",
            headers={"WWW-Authenticate": "Bearer"},
        )
    except JWTClaimsError as e:
        raise HTTPException(status_code=401, detail=f"Invalid JWT claims: {e}")
    except (JWKError, JOSEError) as e:
        raise HTTPException(status_code=401, detail=f"JWT verification failed: {e}")
    except Exception as e:
        raise HTTPException(status_code=401, detail=f"JWT decode error: {e}")

    sub = payload.get("sub")
    if not sub:
        raise HTTPException(status_code=401, detail="Invalid token: sub missing")

    roles = payload.get("roles") or []
    if not isinstance(roles, list):
        roles = [str(roles)]

    return TokenData(sub=sub, roles=roles)