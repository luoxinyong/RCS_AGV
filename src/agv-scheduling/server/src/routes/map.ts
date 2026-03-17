import { Router } from "express";
import { getMaps, getMap, editMap } from "../controllers/map";
import { validate } from "../middlewares/validate";
import { searchMapSchema, editMapSchema } from "../validators";

const router = Router();

router.get("/all", getMaps);
router.get("/find", validate(searchMapSchema, "query"), getMap);
router.post("/edit", validate(editMapSchema), editMap);

export default router;
