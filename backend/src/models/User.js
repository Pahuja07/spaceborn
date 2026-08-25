import mongoose  from 'mongoose';
import bcrypt from 'bcrypt'
import jwt from "jsonwebtoken"



const userSchema = new mongoose.Schema({
  name: { type: String, required: true },
  email: { type: String, required: true, unique: true },
  password: { type: String, required: true },
  role: { type: String, default: 'user' },
  apiKey: { type: String, default: null },
}, { timestamps: true });

// MUST export the compiled model as default


// Hash password before saving
// ✅ CORRECT: Async middleware doesn't need 'next'
userSchema.pre('save', async function () {
  if (!this.isModified('password')) {
    return; // Simply return when not modified
  }
  const salt = await bcrypt.genSalt(10);
  this.password = await bcrypt.hash(this.password, salt);
  // No next() call needed! Returning resolves the promise.
});

// Compare password
userSchema.methods.comparePassword = async function (candidatePassword) {
  return await bcrypt.compare(candidatePassword, this.password);
};

// Generate JWT
userSchema.methods.generateToken = function () {
  return jwt.sign({ id: this._id }, process.env.JWT_SECRET, {
    expiresIn: '7d'
  });
};

const User = mongoose.model('User', userSchema);

export default User;
   