import { Router } from "express";
import { login } from "../controllers/user";
import { validate } from "../middlewares/validate";
import { loginSchema } from "../validators";

const router = Router();

router.post("/login", validate(loginSchema), login);

export default router;
