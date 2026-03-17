import { Router } from "express";
import { getAlarms } from "../controllers/alarms";
import { validate } from "../middlewares/validate";
import { paginationSchema } from "../validators";

const router = Router();

router.get("/", validate(paginationSchema, "query"), getAlarms);

export default router;
