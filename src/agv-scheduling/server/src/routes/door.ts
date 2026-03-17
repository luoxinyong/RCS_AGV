import { Router } from "express";
import { getDoors } from "../controllers/door";

const router = Router();

router.get("/", getDoors);

export default router;
