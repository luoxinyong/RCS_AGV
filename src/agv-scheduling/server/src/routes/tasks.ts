import { Router } from "express";
import { getTaskLogs } from "../controllers/tasks";
import { validate } from "../middlewares/validate";
import { paginationSchema } from "../validators";

const router = Router();

router.get("/logs", validate(paginationSchema, "query"), getTaskLogs);

export default router;
