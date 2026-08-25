import User from '../models/User.js';
import bcrypt from 'bcrypt';
import jwt from 'jsonwebtoken';
import crypto from 'crypto';

// Register User
// Inside register in authController.js:
export const register = async (req, res, next) => {
  try {
    const { name, email, password, role } = req.body;

    const existingUser = await User.findOne({ email });
    if (existingUser) return res.status(400).json({ message: 'User already exists' });

    // Pass plain password; userSchema.pre('save') handles the hashing
    const user = await User.create({ name, email, password, role });

    const token = jwt.sign({ id: user._id, role: user.role }, process.env.JWT_SECRET, { expiresIn: '1d' });

    res.status(201).json({ success: true, token, user: { id: user._id, name: user.name, email: user.email, role: user.role } });
  } catch (error) {
    next(error);
  }
};

// Login User
// Inside login in authController.js:
export const login = async (req, res, next) => {
  try {
    const { email, password } = req.body;

    // 1. Check if user exists
    const user = await User.findOne({ email });
    if (!user) {
      return res.status(401).json({ message: 'Invalid credentials' });
    }

    // 2. Compare plain password with stored hash
    const isMatch = await bcrypt.compare(password, user.password);
    if (!isMatch) {
      return res.status(401).json({ message: 'Invalid credentials' });
    }

    const token = jwt.sign({ id: user._id, role: user.role }, process.env.JWT_SECRET, { expiresIn: '1d' });

    res.status(200).json({ success: true, token, user: { id: user._id, name: user.name, email: user.email, role: user.role } });
  } catch (error) {
    next(error);
  }
};
// Get Currently Authenticated User Profile
export const getMe = async (req, res, next) => {
  try {
    const user = await User.findById(req.user.id).select('-password');
    res.status(200).json({ success: true, data: user });
  } catch (error) {
    next(error);
  }
};

// Generate API Key for External Simulator Access
export const generateApiKey = async (req, res, next) => {
  try {
    const apiKey = `spb_sim_${crypto.randomBytes(24).toString('hex')}`;
    const user = await User.findByIdAndUpdate(req.user.id, { apiKey }, { new: true });

    res.status(200).json({ 
      success: true, 
      message: 'Simulator API Key generated successfully', 
      apiKey: user.apiKey 
    });
  } catch (error) {
    next(error);
  }
};
