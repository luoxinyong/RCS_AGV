import { Router } from "express";
import { validate } from "../middlewares/validate";
import { createDeviceSchema, editDeviceSchema } from "../validators";
import { getDevices, createDevice, editDevice } from "../controllers/device";

const router = Router();

router.get("/", getDevices);
router.post("/add", validate(createDeviceSchema), createDevice);
router.post("/edit", validate(editDeviceSchema), editDevice);

export default router;
