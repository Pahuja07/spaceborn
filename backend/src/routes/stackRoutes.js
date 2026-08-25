import express from "express"
const router = express.Router();
import { 
  getAllStacks, 
  getStackById, 
  createStack 
} from '../controllers/stackController.js';

router.route('/')
  .get(getAllStacks)
  .post(createStack);

router.route('/:id')
  .get(getStackById);

export default router
