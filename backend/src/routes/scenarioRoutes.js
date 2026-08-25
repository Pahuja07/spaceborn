import express from "express"
const router = express.Router();
import { 
  getAllScenarios, 
  getScenarioById, 
  createScenario 
} from   '../controllers/scenarioController.js';

router.route('/')
  .get(getAllScenarios)
  .post(createScenario);

router.route('/:id')
  .get(getScenarioById);


  export default router